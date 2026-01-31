
import customtkinter as ctk # GUI
import serial # Serial communication
import serial.tools.list_ports # Serial port listing
import threading # Threading for background tasks
import time # Time for sleep and timing
import csv # CSV file handling
import numpy as np # Numpy for data processing
import matplotlib.pyplot as plt # Matplotlib for plotting
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg # Matplotlib for tkinter
from tkinter import filedialog, messagebox # Tkinter for file dialogs and message boxes
from scipy.optimize import curve_fit # Scipy for curve fitting

# Configuration
ctk.set_appearance_mode("System") # GUI appearance
ctk.set_default_color_theme("blue") # GUI color theme

class EFNMRApp(ctk.CTk):
    def __init__(self):
        super().__init__() # Initialize the parent class

        self.title("C++ NMR Spectrometer Controller") # Window title
        self.geometry("1200x800") # Window size
        
        self.serial_port = None # Serial port
        self.is_connected = False # Connection status
        self.is_mock = False # Mock mode
        self.stop_event = threading.Event() # Stop event

        self.setup_ui() # Setup the UI
        
    def setup_ui(self):
        # Layout: Left Sidebar (Controls), Right (Plots)
        self.grid_columnconfigure(1, weight=1) # Right column weight
        self.grid_rowconfigure(0, weight=1) # Top row weight

        # Sidebar
        self.sidebar = ctk.CTkFrame(self, width=300, corner_radius=0) # Sidebar frame
        self.sidebar.grid(row=0, column=0, sticky="nsew") # Sidebar grid
        
        # Connection Frame
        self.conn_frame = ctk.CTkFrame(self.sidebar) # Connection frame
        self.conn_frame.pack(pady=10, padx=10, fill="x") # Connection frame packing
        
        ctk.CTkLabel(self.conn_frame, text="Connection", font=("Roboto", 16, "bold")).pack(pady=5) # Connection label
        
        self.port_combo = ctk.CTkComboBox(self.conn_frame, values=self.get_ports()) # Port combo box
        self.port_combo.pack(pady=5, padx=5, fill="x") # Port combo box packing
        self.port_combo.set("Select Port") # Default empty
        
        self.btn_refresh = ctk.CTkButton(self.conn_frame, text="Refresh Ports", command=self.refresh_ports) # Refresh ports button
        self.btn_refresh.pack(pady=5, padx=5)
        
        self.mock_switch = ctk.CTkSwitch(self.conn_frame, text="Mock Device Mode", command=self.toggle_mock) # Mock switch
        self.mock_switch.pack(pady=5)
        
        self.btn_connect = ctk.CTkButton(self.conn_frame, text="Connect", command=self.toggle_connection, fg_color="green") # Connect button
        self.btn_connect.pack(pady=10, padx=5, fill="x")

        # Command Parameters
        self.param_frame = ctk.CTkFrame(self.sidebar) # Command parameters frame
        self.param_frame.pack(pady=10, padx=10, fill="x") # Command parameters frame packing
        
        ctk.CTkLabel(self.param_frame, text="Parameters", font=("Roboto", 16, "bold")).pack(pady=5) # Parameters label
        
        self.create_param_entry("Sleep Time (us):", "20") # Sleep time entry
        self.create_param_entry("Data Size:", "10000") # Data size entry
        self.create_param_entry("Tau (us):", "5000") # Tau entry
        self.create_param_entry("Num Echoes:", "20") # Num echoes entry

        # Command Buttons
        self.btn_fid = ctk.CTkButton(self.sidebar, text="Run FID", command=self.run_fid) # Run FID button
        self.btn_fid.pack(pady=5, padx=10, fill="x")
        
        self.btn_cpmg = ctk.CTkButton(self.sidebar, text="Run CPMG/Echo", command=self.run_cpmg) # Run CPMG button
        self.btn_cpmg.pack(pady=5, padx=10, fill="x")
        
        self.btn_export = ctk.CTkButton(self.sidebar, text="Export Data", command=self.export_data, state="disabled") # Export data button
        self.btn_export.pack(pady=20, padx=10, fill="x")

        # Plots Area (Tabs)
        self.tab_view = ctk.CTkTabview(self) # Tab view
        self.tab_view.grid(row=0, column=1, padx=10, pady=10, sticky="nsew") # Tab view grid
        
        self.tab_raw = self.tab_view.add("Raw Signal") # Raw signal tab
        self.tab_t2 = self.tab_view.add("T2 Analysis") # T2 analysis tab
        
        # Raw Plot
        self.fig_raw, self.ax_raw = plt.subplots(figsize=(5, 4), dpi=100) # Raw plot
        self.canvas_raw = FigureCanvasTkAgg(self.fig_raw, master=self.tab_raw) # Raw plot canvas
        self.canvas_raw.get_tk_widget().pack(fill="both", expand=True) # Raw plot canvas packing
        self.ax_raw.set_title("ADC Values vs Time") # Raw plot title
        self.ax_raw.set_xlabel("Time (us)") # Raw plot x label
        self.ax_raw.set_ylabel("ADC Values") # Raw plot y label (1 ADC unit = 0.8 mV for Pico)
        self.ax_raw.grid(True)

        # T2 Plot
        self.fig_t2, self.ax_t2 = plt.subplots(figsize=(5, 4), dpi=100) # T2 plot
        self.canvas_t2 = FigureCanvasTkAgg(self.fig_t2, master=self.tab_t2) # T2 plot canvas
        self.canvas_t2.get_tk_widget().pack(fill="both", expand=True) # T2 plot canvas packing
        self.ax_t2.set_title("T2 Relaxation Analysis") # T2 plot title
        self.ax_t2.set_xlabel("Time (ms)") # T2 plot x label
        self.ax_t2.set_ylabel("Peak Amplitude") # T2 plot y label
        self.ax_t2.grid(True)
        
        self.data_time = [] # Data time
        self.data_values = [] # Data ADC values
        self.t2_time = [] # T2 time
        self.t2_amp = [] # T2 amplitude

    def create_param_entry(self, label, default): # Create parameter entry
        f = ctk.CTkFrame(self.param_frame, fg_color="transparent") 
        f.pack(fill="x", pady=2)
        ctk.CTkLabel(f, text=label, width=100, anchor="w").pack(side="left")
        e = ctk.CTkEntry(f, width=80)
        e.pack(side="right")
        e.insert(0, default)
        
        # Store reference by label
        setattr(self, f"entry_{label.split()[0].lower()}", e)

    def get_ports(self): # Get ports
        return [p.device for p in serial.tools.list_ports.comports()]
    
    def refresh_ports(self): # Refresh ports
        self.port_combo.configure(values=self.get_ports())

    def toggle_mock(self): # Toggle mock mode
        self.is_mock = self.mock_switch.get() == 1
        if self.is_mock:
            self.port_combo.configure(state="disabled")
            self.btn_refresh.configure(state="disabled")
        else:
            self.port_combo.configure(state="normal")
            self.btn_refresh.configure(state="normal")

    def toggle_connection(self): # Toggle connection
        if self.is_connected: # If connected
            if self.serial_port: # If serial port is open
                self.serial_port.close() # Close serial port
            self.is_connected = False # Set connected to false
            self.btn_connect.configure(text="Connect", fg_color="green") # Set button text to connect
            self.sidebar.configure(border_color="gray") # Set sidebar border color to gray
        else:
            try:
                if not self.is_mock: # If not mock mode
                    port = self.port_combo.get() # Get port
                    if not port: return # If no port selected
                    self.serial_port = serial.Serial(port, 115200, timeout=1) # Open serial port
                self.is_connected = True # Set connected to true
                self.btn_connect.configure(text="Disconnect", fg_color="red") # Set button text to disconnect
            except Exception as e:
                messagebox.showerror("Connection Error", str(e)) # Show error message

    def run_fid(self): # Run FID
        self.run_command("FID")

    def run_cpmg(self): # Run CPMG
        self.run_command("CPMG")

    def run_command(self, cmd_type): # Run command
        if not self.is_connected and not self.is_mock: # If not connected and not mock mode
            messagebox.showwarning("Warning", "Not connected!") # Show warning message
            return

        # Get Parameters
        try:
            sleep = int(self.entry_sleep.get()) # Get sleep
            dsize = int(self.entry_data.get()) # Get data size
            tau = int(self.entry_tau.get()) # Get tau
            echoes = int(self.entry_num.get()) # Get echoes
        except ValueError:
            messagebox.showerror("Error", "Invalid parameters") # Show error message
            return

        # Layout cmd: "TYPE,sleep,size,tau,echoes"
        cmd_str = f"{cmd_type},{sleep},{dsize},{tau},{echoes}\n"
        
        if self.is_mock: # If mock mode
            threading.Thread(target=self.mock_receive, args=(cmd_type, sleep, dsize, tau, echoes)).start()
        else: # If not mock mode
            self.serial_port.write(cmd_str.encode()) # Send command
            threading.Thread(target=self.serial_receive).start()

# CREATING MOCK DATA IF SELECTED MOCK DEVICE MODE
    def mock_receive(self, cmd_type, sleep, dsize, tau, echoes):
        # Generate fake decaying sine/echo train
        self.data_time = [] # Data time
        self.data_values = [] # Data ADC values
        
        t = np.linspace(0, dsize * sleep, dsize) # Time
        
        if cmd_type == "FID": # If Mock FID
            # Decaying exponential sine
            y = 2000 * np.exp(-t/10000) * np.sin(2 * np.pi * 0.002 * t) + 2048 # Mock FID Curve Data
        else: # If Mock CPMG
            # CPMG: Series of echoes
            y = np.ones_like(t) * 2048 # Baseline
            # Add echoes at 2*tau*n
            # very rough simulation of echo train
            for n in range(1, echoes + 1):
                echo_time_us = (2 * tau * n) # Echo time
                # Find index relative to time
                # Simple envelope: Gaussian at echo time ~ T2
                envelope = 1000 * np.exp(-(n * 2 * tau) / 50000) # Decay T2 ~ 50ms
                
                # Each echo is a Gaussian blob
                blob = envelope * np.exp(-0.5 * ((t - echo_time_us)/100)**2) # Echo blob
                y += blob
                
            # Add noise
            y += np.random.normal(0, 10, dsize) # Add Mock Noise

        self.data_time = t.tolist() # Add mock time data to list
        self.data_values = y.tolist() # Add mock ADC values data to list
        
        self.after(0, self.update_plots, cmd_type) # Update plots

# READING REAL DATA IF CONNECTED TO RASPBERRY PI PICO
    def serial_receive(self): 
        self.data_time = [] # Data time
        self.data_values = [] # Data ADC values
        
        # Read lines until timeout or done?
        # Firmware sends line by line: "time,val"
        # We need a robust reader.
        # Ideally, firmware signals 'START' and 'END'. 
        # Current firmware just starts printing CSV.
        # Room for improvement.
        
        start_time = time.time()
        while time.time() - start_time < 10: # 10s timeout 
            if self.serial_port.in_waiting: # If data available
                line = self.serial_port.readline().decode().strip() # Read line
                if not line: continue # Skip empty lines
                if "," in line: # If comma in line
                    try:
                        t, v = map(float, line.split(",")) # Split line
                        self.data_time.append(t) # Add time
                        self.data_values.append(v) # Add ADC value
                    except:
                        pass # Skip invalid lines
                start_time = time.time() # Reset timeout on data
        
        self.after(0, self.update_plots, "CPMG") # Assume CPMG

    def update_plots(self, mode): # Update plots
        self.btn_export.configure(state="normal") # Enable export button
        
        # Raw Plot
        self.ax_raw.clear() # Clear raw plot
        self.ax_raw.plot(self.data_time, self.data_values, color='#4a90e2') # Plot raw data
        self.ax_raw.set_title("ADC Values vs Time") # Set title
        self.ax_raw.set_xlabel("Time (us)") # Set x label
        self.ax_raw.set_ylabel("ADC Values") # Set y label
        self.ax_raw.grid(True, alpha=0.3) # Add grid
        self.canvas_raw.draw() # Draw canvas
        
        # T2 Analysis if CPMG mode
        if mode == "CPMG" and len(self.data_values) > 0: # If CPMG mode and data available
            self.analyze_t2()
            
    def analyze_t2(self): # Analyze T2
        # Extract peaks from echo train
        # Simple algorithm: Find local maxima in windows expected by Tau
        try:
            tau = int(self.entry_tau.get()) # Get tau
            echoes = int(self.entry_num.get()) # Get echoes
            
            peaks_t = [] # Peaks time
            peaks_v = [] # Peaks ADC values
            
            arr_t = np.array(self.data_time) # Data time
            arr_v = np.array(self.data_values) # Data ADC values
            baseline = 2048 # ADC mid-point approximately
            
            # Refine baseline
            baseline = np.min(arr_v) 
            
            for n in range(1, echoes + 1): # For each echo
                center_time = 2 * tau * n # Center time
                # Search window +/- tau/2
                mask = (arr_t > center_time - tau/2) & (arr_t < center_time + tau/2)
                if np.any(mask): # If mask is not empty
                    window_v = arr_v[mask] # Window ADC values
                    window_t = arr_t[mask] # Window time
                    
                    peak_idx = np.argmax(window_v) # Peak index
                    peak_val = window_v[peak_idx] - baseline # Peak ADC value
                    peak_time = window_t[peak_idx] # Peak time
                    
                    peaks_t.append(peak_time / 1000.0) # Convert to ms
                    peaks_v.append(peak_val) # Append peak ADC value
            
            self.ax_t2.clear() # Clear T2 plot
            self.ax_t2.scatter(peaks_t, peaks_v, color='red', label='Echo Peaks') # Scatter peaks
            
            # Fit exponential decay: V = A * exp(-t/T2)
            if len(peaks_t) > 2: # If more than 2 peaks
                def decay(t, a, t2): # Decay function
                    return a * np.exp(-t / t2)
                
                try:
                    popt, _ = curve_fit(decay, peaks_t, peaks_v, p0=[max(peaks_v), 10.0]) # Curve fit
                    t2_val = popt[1] # T2 value
                    
                    fit_t = np.linspace(min(peaks_t), max(peaks_t), 100) # Fit time
                    fit_v = decay(fit_t, *popt) # Fit ADC values
                    
                    self.ax_t2.plot(fit_t, fit_v, 'g--', label=f'Fit: T2={t2_val:.2f} ms') # Plot fit
                    self.t2_time = peaks_t # T2 time
                    self.t2_amp = peaks_v # T2 amplitude
                except: # If fit fails
                    print("Fit failed")
            
            self.ax_t2.legend() # Add legend
            self.ax_t2.grid(True, alpha=0.3) # Add grid
            self.ax_t2.set_title("T2 Relaxation Analysis") # Set title
            self.ax_t2.set_xlabel("Time (ms)") # Set x label
            self.ax_t2.set_ylabel("Peak Amplitude") # Set y label
            self.canvas_t2.draw() # Draw canvas
            
        except Exception as e: # If error
            print(f"T2 Analysis Error: {e}") # Print error

    def export_data(self): # Export data
        filename = filedialog.asksaveasfilename(defaultextension=".csv") # Ask for filename
        if filename: # If filename is not empty
            with open(filename, 'w', newline='') as f: # Open file
                writer = csv.writer(f) # CSV writer
                writer.writerow(["Time_us", "ADC_Values"]) # Write header
                for t, v in zip(self.data_time, self.data_values): # For each data point
                    writer.writerow([t, v]) # Write data point
            
            # Save T2 data if exists
            if hasattr(self, 't2_time') and len(self.t2_time) > 0: # If T2 data exists
                with open(filename.replace(".csv", "_T2.csv"), 'w', newline='') as f: # Open file
                    writer = csv.writer(f) # CSV writer
                    writer.writerow(["Time_ms", "Peak_Amplitude"]) # Write header
                    for t, v in zip(self.t2_time, self.t2_amp): # For each data point
                        writer.writerow([t, v]) # Write data point

if __name__ == "__main__": # If main
    app = EFNMRApp() # Create app
    app.mainloop() # Run app
