
#include "Arduino.h"        // Arduino
#include "hardware/adc.h"   // ADC
#include "hardware/dma.h"   // DMA
#include "hardware/irq.h"   // IRQ
#include "hardware/pio.h"   // PIO
#include "pico/multicore.h" // Multicore
#include "pio_pulses.pio.h" // PIO Pulses from Earle Philhower's RP2040 core

/*
   EFNMR Spectrometer Controller
   Platform: Raspberry Pi Pico 2 (RP2350) / RP2040

   Optimisations from previous iterations:
   - PIO: Cycle-accurate CPMG pulse sequences (No CPU Jitter).
   - DMA: Background ADC transfer (Zero CPU overhead during acquisition).
*/

// --- Pin Definitions ---
#define LED_PIN 25        // On-board LED
#define ADC_PIN 28        // ADC2 (GP28)
#define PULSE_PIN 16      // Pulse Output Pin
#define PP_COIL_PIN 26    // Pre-Polarization Coil Control Pin
#define DET_SWITCH_PIN 22 // Detection Switch Control Pin

// --- Constants ---
const float LARMOR_FREQ = 2210.0; // Hz (Adjustable based on field strength)
// Note: User can update these via Serial commands if needed

// --- Global Objects ---
// PIO Globals
PIO pio_block = pio0; // PIO Block
uint sm = 0;          // State Machine
uint offset = 0;      // PIO Offset

// DMA Globals
int dma_chan;               // DMA Channel
dma_channel_config dma_cfg; // DMA Configuration

// Buffers
// Max samples per echo window? Assume reasonable buffer size.
// If datasize is huge, we might need double buffering. For now, massive buffer.
#define MAX_SAMPLES 10000               // Maximum samples per echo window
uint16_t adc_buffer[MAX_SAMPLES];       // ADC Buffer
volatile int samples_collected = 0;     // Samples Collected
volatile bool acquisition_done = false; // Acquisition Done

// Variables
int g_sleep_time = 20;   // Sampling interval (us) / Noise attenuation delay
int g_datasize = 100;    // Data size
int g_turn_on_time = 20; // Turn on time

// --- Function Prototypes ---
void setup_pio();                                          // PIO Setup
void setup_dma();                                          // DMA Setup
void trigger_cpmg(int n_echoes, int pulse_us, int tau_us); // Trigger CPMG
void fid_command(int sleep_time, int datasize);            // FID Command
void cpmg_command(int sleep_time, int datasize, int tau_us,
                  int n_echoes); // CPMG Command

// --- Setup ---
void setup() {
  Serial.begin(115200); // Increased baud rate for CSV dump

  pinMode(LED_PIN, OUTPUT);        // LED Pin
  pinMode(PULSE_PIN, OUTPUT);      // Pulse Pin
  pinMode(PP_COIL_PIN, OUTPUT);    // Pre-Polarization Coil Pin
  pinMode(DET_SWITCH_PIN, OUTPUT); // Detection Switch Pin

  // ADC Setup
  adc_init();
  adc_gpio_init(ADC_PIN); // ADC Pin
  adc_select_input(2);    // GP28 is ADC 2
  adc_fifo_setup(true,    // Write to FIFO
                 true,    // DMA request enabled
                 1,       // DREQ threshold
                 false,   // No error bit
                 false    // Keep 12-bit
  );                      // ADC FIFO Setup

  setup_pio(); // PIO Setup
  setup_dma(); // DMA Setup

  Serial.println("EFNMR Optimized Controller Ready (PIO+DMA)"); // Ready
}

// --- PIO Setup ---
void setup_pio() {
  // Load the CPMG program into PIO
  if (pio_can_add_program(pio_block,
                          &cpmg_program)) { // Check if PIO can add program
    offset = pio_add_program(pio_block, &cpmg_program); // Add program to PIO
  } else {
    Serial.println("Error: Could not add PIO program!"); // Error handling
    return;
  }

  sm = pio_claim_unused_sm(pio_block, true); // Claim unused SM

  // Custom init function since we rely on manual header
  pio_sm_config c =
      cpmg_program_get_default_config(offset); // Get default config
  sm_config_set_sideset_pins(&c, PULSE_PIN);   // Side set affects Pulse Pin
  sm_config_set_out_pins(&c, PULSE_PIN, 1);    // Set as output
  pio_gpio_init(pio_block, PULSE_PIN);         // Initialize GPIO pin
  pio_sm_set_consecutive_pindirs(pio_block, sm, PULSE_PIN, 1,
                                 true); // Set pin direction

  // Shift configuration
  // Auto-pull disabled (we pull manually in ASM), Join FIFO disabled (we need
  // separate TX/RX typically, but here mostly TX)
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX); // Set FIFO join

  // Clock divider to run PIO at 1MHz (1us/cycle) for easy timing?
  // Or run at full speed and calculate large delays.
  // Full speed (125MHz) is better for precision.
  // We use sys_clock in calculations, so no divider needed strictly, but
  // simplifies math if 1 cycle = X time. Let's stick to full speed.
  sm_config_set_clkdiv(&c, 1.0f); // Set clock divider

  pio_sm_init(pio_block, sm, offset, &c); // Initialize SM
}

// --- DMA Setup ---
void setup_dma() {
  dma_chan = dma_claim_unused_channel(true);          // Claim unused channel
  dma_cfg = dma_channel_get_default_config(dma_chan); // Get default config

  channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // Set data size
  channel_config_set_read_increment(&dma_cfg,
                                    false); // Read from fixed ADC FIFO
  channel_config_set_write_increment(&dma_cfg, true); // Write to buffer
  channel_config_set_dreq(&dma_cfg, DREQ_ADC);        // Paced by ADC
}

// --- Helper: Calculate Clock Div for ADC Rate ---
void start_adc_dma(int count, int interval_us) {
  // Set ADC clock divider to match interval
  // interval_us = (1 + div) / 48MHz
  // div = interval_us * 48 - 1
  // Min sampling time is 2us (500ksps)
  if (interval_us < 2) {
    interval_us = 2;
  }

  adc_set_clkdiv((float)interval_us * 48.0 - 1.0); // Set clock divider

  // Drain FIFO
  while (!adc_fifo_is_empty()) {
    adc_fifo_get();
  }

  dma_channel_configure(dma_chan, &dma_cfg,
                        adc_buffer,    // Destination
                        &adc_hw->fifo, // Source
                        count,         // Count
                        true           // Start immediately
  );

  adc_run(true); // Run ADC
}

// --- Main Loop (Command Processor) ---
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read command
    command.trim();
    if (command.length() == 0)
      return;

    // Simple parser
    int firstComma = command.indexOf(','); // Find first comma
    String cmdType = (firstComma == -1)
                         ? command
                         : command.substring(0, firstComma); // Command type
    String params = (firstComma == -1)
                        ? ""
                        : command.substring(firstComma + 1); // Parameters

    // Parse params into array
    long args[5] = {0, 0, 0, 0, 0}; // Initialize array
    int arg_count = 0;              // Argument count
    int ptr = 0;                    // Pointer
    while (ptr < params.length() && arg_count < 5) {
      int nextComma = params.indexOf(',', ptr); // Next comma
      if (nextComma == -1)
        nextComma = params.length(); // No more commas
      args[arg_count++] =
          params.substring(ptr, nextComma).toInt(); // Convert to int
      ptr = nextComma + 1;                          // Move pointer
    }

    if (cmdType == "FID") {
      // Args: sleep_time, datasize
      fid_command(args[0], args[1]); // Call FID command
    } else if (cmdType == "CPMG" || cmdType == "echo") {
      // Args: sleep_time, datasize, tau_us, n_echoes
      // 'echo' mapped to similar logic
      cpmg_command(args[0], args[1], args[2], args[3]); // Call CPMG command
    } else {
      Serial.println("Unknown command"); // Error handling
    }
  }
}

// --- Command Implementations ---

// --- CPMG Command ---
void cpmg_command(int sleep_time, int datasize, int tau_us, int n_echoes) {
  // 1. Pre-Polarization
  digitalWrite(LED_PIN, HIGH);     // Turn on LED
  digitalWrite(PP_COIL_PIN, HIGH); // Turn on PP coil
  delay(3000);                     // 3 seconds charge
  digitalWrite(PP_COIL_PIN, LOW);  // Turn off PP coil

  // 2. Configure PIO for CPMG
  // 90 pulse = 1/4 period.

  float period = 1000000.0 / LARMOR_FREQ; // Period in us
  int pulse_time = (int)(period / 4.0);   // 90 pulse time in us

  uint32_t sys_freq = clock_get_hz(clk_sys); // System frequency in Hz
  uint32_t cycles_p90 =
      (uint32_t)(pulse_time * (sys_freq / 1000000.0)); // 90 pulse cycles
  uint32_t cycles_tau =
      (uint32_t)(tau_us * (sys_freq / 1000000.0)); // Tau cycles
  uint32_t cycles_p180 = cycles_p90 * 2;           // 180 is double length of 90

  // 3. Start Sequence
  digitalWrite(DET_SWITCH_PIN, HIGH); // Enable RX
  delayMicroseconds(g_turn_on_time);  // Wait for switch

  // Clear and fill PIO FIFO
  pio_sm_clear_fifos(pio_block, sm);

  // Order matches .pio program:
  // 1. 90_len
  pio_sm_put_blocking(pio_block, sm, cycles_p90);
  // 2. Tau_len (First delay)
  pio_sm_put_blocking(pio_block, sm, cycles_tau);
  // 3. Loop Count (N Echoes)
  pio_sm_put_blocking(pio_block, sm, n_echoes);
  // 4. 180_len
  pio_sm_put_blocking(pio_block, sm, cycles_p180);
  // 5. Tau_len (Loop delay)
  pio_sm_put_blocking(pio_block, sm, cycles_tau);

  // Continuous DMA stream for the duration of the sequence.
  int total_duration_us =
      pulse_time + tau_us +
      n_echoes * (2 * pulse_time + 2 * tau_us);       // Total duration in us
  int total_samples = total_duration_us / sleep_time; // Total samples
  if (total_samples > MAX_SAMPLES)
    total_samples = MAX_SAMPLES; // Limit samples

  start_adc_dma(total_samples, sleep_time); // Start ADC DMA

  // Enable PIO
  pio_sm_set_enabled(pio_block, sm, true);

  // Wait for DMA to finish
  dma_channel_wait_for_finish_blocking(dma_chan);

  // Stop everything
  pio_sm_set_enabled(pio_block, sm, false); // Disable PIO
  adc_run(false);                           // Stop ADC
  digitalWrite(DET_SWITCH_PIN, LOW);        // Disable RX
  digitalWrite(LED_PIN, LOW);               // Turn off LED

  // 4. Send Data
  // Output: Time(us), Value
  for (int i = 0; i < total_samples; i++) {
    Serial.print(i * sleep_time); // Time in us
    Serial.print(",");
    Serial.println(adc_buffer[i]); // ADC Value
  }
}

// --- FID Command ---
void fid_command(int sleep_time, int datasize) {
  // Similar to CPMG but 1 pulse
  digitalWrite(LED_PIN, HIGH);     // Turn on LED
  digitalWrite(PP_COIL_PIN, HIGH); // Turn on PP coil
  delay(3000);                     // 3 seconds charge
  digitalWrite(PP_COIL_PIN, LOW);  // Turn off PP coil

  // Pulse
  float period = 1000000.0 / LARMOR_FREQ; // Period in us
  int pulse_time = (int)(period / 4.0);   // 90 pulse time in us

  // Manual bit bang for single pulse is fine, PIO is overkill, plus can see if
  // bit bang is even working
  digitalWrite(PULSE_PIN, HIGH); // Turn on pulse
  delayMicroseconds(pulse_time); // Pulse duration
  digitalWrite(PULSE_PIN, LOW);  // Turn off pulse

  digitalWrite(DET_SWITCH_PIN, HIGH); // Enable RX
  // DMA Capture
  start_adc_dma(datasize, sleep_time);
  dma_channel_wait_for_finish_blocking(dma_chan); // Wait for DMA to finish
  adc_run(false);                                 // Stop ADC

  digitalWrite(DET_SWITCH_PIN, LOW); // Disable RX
  digitalWrite(LED_PIN, LOW);        // Turn off LED

  for (int i = 0; i < datasize; i++) {
    Serial.print(i * sleep_time); // Time in us
    Serial.print(",");
    Serial.println(adc_buffer[i]); // ADC Value
  } // Output: Time(us), Value
}
