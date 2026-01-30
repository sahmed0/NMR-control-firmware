# Earth's Field NMR Spectrometer Controller (EFNMR)

A high-precision, low-cost controller for Earth's Field Nuclear Magnetic Resonance (EFNMR) spectroscopy, built on the **Raspberry Pi Pico (RP2040/RP2350)**.

This project uses Pico's advanced **PIO (Programmable I/O)** and **DMA (Direct Memory Access)** to generate cycle-accurate pulse sequences (CPMG, FID) and to capture high-speed ADC data simultaneously, without CPU overhead.

## 🚀 Features

*   **Cycle-Accurate Timing:** Uses dedicated PIO hardware to generate RF pulses with microsecond precision, completely independent of CPU jitter.
*   **Background Data Acquisition:** Uses DMA to stream ADC data directly to memory, ensuring no samples are missed during complex pulse sequences.
*   **User-Friendly GUI:** A modern Python interface (CustomTkinter) to control parameters, visualise echoes in real-time, and perform T2 relaxation analysis.
*   **Supported Sequences:**
    *   **FID:** Free Induction Decay (Single Pulse).
    *   **CPMG:** Carr-Purcell-Meiboom-Gill (Spin Echo Train) for T2 measurement.

## User Interface
### Free Induction Decay
![free induction decay](images/FID.png)

### Carr-Purcell-Meiboom-Gill Pulse Sequence
![CPMG pulse sequence](images/CPMG.png)

### Transverse ($$T2$$) Relaxation Time
![transverse relaxation time](images/T2.png)

## 🛠️ Hardware Setup

The system is designed for a Raspberry Pi Pico.

| Pin | Function | Description |
| :--- | :--- | :--- |
| **GP16** | `PULSE_PIN` | MOSFET Driver for the RF Transmit Coil (B1 field) |
| **GP26** | `PP_COIL_PIN` | Relay/Switch for Pre-Polarisation Coil (Bp field) |
| **GP22** | `DET_SWITCH_PIN` | RX Isolation Switch (Protects receiver during pulses) |
| **GP28** | `ADC_PIN` | Analog Input from the Receiver (Pre-amp output) |
| **GP25** | `LED_PIN` | On-board Status LED |

> **Note:** The Larmor frequency is configured for approx **2.2 kHz** (standard Earth's Field) but can be tuned in `main.ino`.

## 📦 Software Installation

### 1. Firmware (Raspberry Pi Pico)
1.  Install the **Arduino IDE**.
2.  Install the **Raspberry Pi Pico Arduino Core** by Earle F. Philhower (search for "Pico" in Board Manager).
3.  Select your board (Raspberry Pi Pico or Pico 2).
4.  Open `main.ino` and upload it to the board.

### 2. GUI (Computer)
Ensure you have Python installed. Install the required dependencies:

```bash
pip install customtkinter pyserial numpy matplotlib scipy
```

## 🖥️ Usage

1.  **Connect** the Pico to your computer via USB.
2.  **Run the GUI**:
    ```bash
    python gui/gui.py
    ```
3.  **Control Panel**:
    *   **Port:** Select the COM port of your Pico.
    *   **Connect:** Click to establish serial communication.
    *   **Parameters:**
        *   `Sleep Time`: Sampling interval in µs (inverse of sample rate).
        *   `Data Size`: Number of points to capture.
        *   `Tau`: Delay between pulses (half-echo spacing).
    *   **Num Echoes**: Number of 180° pulses in the train.
    *   **Run:** Click **Run FID** or **Run CPMG**.
4.  **Export:** Save your data to CSV for further processing.

## 🔬 How It Works (The "Secret Sauce")

### The PIO Engine (`pio_pulses.pio`)
Instead of using `delay()` or interrupts, which are inaccurate, we write a custom assembly program for the Pico's PIO state machines. This program sits in the background and waits for commands. When we say "Go", it executes the entire Pulse-Delay-Pulse sequence with 125MHz precision.

### The DMA Engine
While the PIO is blasting RF pulses, the ADC is running at full speed. We use a DMA channel to silently read every byte from the ADC and store it in a RAM buffer. The main CPU does nothing but wait for the "Done" signal!

## 📄 License
Copyright © 2026 Sajid Ahmed. All Rights Reserved.

This repository is intended solely for portfolio review and recruitment purposes. By accessing this repository, you acknowledge the following terms:
- View Only: Permission is granted to view the source code for the purpose of evaluating my professional skills and experience.
- No Unauthorised Use: No permission is granted to copy, modify, distribute, or use this code for any personal, commercial, or educational project.
- No AI Training: Use of this source code for the purpose of training machine learning models or generative AI is strictly prohibited.
