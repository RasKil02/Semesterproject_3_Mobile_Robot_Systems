import numpy as np
import matplotlib

# Force interactive backend so the plot shows
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
from scipy.signal import butter, sosfreqz

# --- Filter parameters ---
fs = 44100       # Sampling frequency
low = 650        # Lower cutoff frequency
high = 1640      # Upper cutoff frequency
order = 4       # Filter order

# --- Design the filter ---
nyq = fs / 2
wn = [low / nyq, high / nyq]

# Second-order sections representation
sos = butter(order, wn, btype='bandpass', output='sos')

# --- Compute frequency response ---
w, h = sosfreqz(sos, worN=4096, fs=fs)   # w = frequency in Hz

# Convert magnitude to dB
h_db = 20 * np.log10(np.abs(h))

# --- Plot ---
plt.figure(figsize=(10, 5))
plt.plot(w, h_db, label="Magnitude Response")
plt.title("Magnitude Response of 4th-Order Butterworth Bandpass Filter")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude [dB]")
plt.grid(True)

# Highlight cutoff frequencies
plt.axvline(low, color='red', linestyle='--', label="620 Hz")
plt.axvline(high, color='green', linestyle='--', label="1700 Hz")

# Zoom into the relevant range
plt.xlim(0, 5000)      # View only 0–5 kHz region
plt.ylim(-80, 5)        # dB scale

plt.legend()

# Show the plot and keep window open
plt.show(block=True)
