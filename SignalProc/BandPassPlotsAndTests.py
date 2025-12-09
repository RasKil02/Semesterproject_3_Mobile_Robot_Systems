import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfilt, sosfreqz, unit_impulse, group_delay, freqz, sos2tf, bode
"""
# ----------------------------
# Filter settings from your DTMF project
# ----------------------------
fs = 44100
lowcut = 620
highcut = 1700
orders = [2, 4, 8, 10]     # orders to compare

nyq = fs / 2
wn = [lowcut/nyq, highcut/nyq]

# ===============================================================
# 1) IMPULSE RESPONSE (shows ringing → why high order is bad)
# ===============================================================
plt.figure(figsize=(10,6))
imp = unit_impulse(44100)   # 1 second impulse

for order in orders:
    sos = butter(order, wn, btype="band", output="sos")
    resp = sosfilt(sos, imp)
    plt.plot(resp[:500], label=f"Order {order}")  # 500 samples ≈ 11 ms

plt.title("Impulse Response of Butterworth Band-pass filter")
plt.xlabel("Sample")
plt.ylabel("Amplitude")
plt.grid(True)
plt.legend()
plt.show()


# ===============================================================
# 2) GROUP DELAY (correct SOS-section-based computation)
# ===============================================================
plt.figure(figsize=(10,6))

# Frequency axis
w = np.linspace(500, 1800, 2000)  # Only plot near DTMF band

for order in orders:
    sos = butter(order, wn, btype="band", output="sos")
    
    total_gd = np.zeros_like(w)

    # Sum group delay of each second-order section
    for section in sos:
        b = section[:3]
        a = section[3:]
        w_sec, gd_sec = group_delay((b, a), w=w, fs=fs)
        total_gd += gd_sec

    plt.plot(w_sec, total_gd, label=f"Order {order}")

plt.axvline(lowcut, color='red', linestyle='--', label="620 Hz")
plt.axvline(highcut, color='green', linestyle='--', label="1700 Hz")

plt.title("Group Delay of Butterworth Band-pass filter")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Group Delay (samples)")
plt.grid(True)
plt.legend()
plt.xlim(500, 1800)
plt.ylim(0, 300)
plt.show()


# ===============================================================
# 3) MAGNITUDE RESPONSE (shows steepness of high orders)
# ===============================================================
plt.figure(figsize=(10, 6))

for order in orders:
    sos = butter(order, wn, btype='band', output='sos')
    w, h = sosfreqz(sos, worN=4000, fs=fs)
    plt.plot(w, 20*np.log10(np.maximum(np.abs(h), 1e-12)),
             label=f"Order {order}")

plt.axvline(lowcut, color='red', linestyle='--', label="620 Hz")
plt.axvline(highcut, color='green', linestyle='--', label="1700 Hz")

plt.title("Magnitude Response for Different Butterworth filter Orders")
plt.xlabel("Frequency [Hz]")
plt.ylabel("Magnitude [dB]")
plt.grid(True)
plt.legend()
plt.ylim([-80, 5])
plt.xlim([0, 5000])
plt.show()


# ===============================================================

order = 4

# Design filter
sos = butter(order, [lowcut/(fs/2), highcut/(fs/2)], btype='bandpass', output='sos')

# Frequency response directly from SOS (correct!)
w, h = sosfreqz(sos, worN=2000, fs=fs)

# Magnitude in dB
mag = 20 * np.log10(np.abs(h))

# Phase in degrees
phase = np.unwrap(np.angle(h)) * 180 / np.pi

# Plot Bode
plt.figure(figsize=(10, 8))

# Magnitude
plt.subplot(2,1,1)
plt.semilogx(w, mag)
plt.grid(True, which='both')
plt.ylabel("Magnitude (dB)")
plt.xlabel("Frequency (Hz)")
plt.title("Bode Plot of 4th Order Butterworth Band-pass Filter")

# Phase
plt.subplot(2,1,2)
plt.semilogx(w, phase)
plt.grid(True, which='both')
plt.xlabel("Frequency (Hz)")
plt.ylabel("Phase (degrees)")

plt.tight_layout()
plt.show()


import numpy as np
import matplotlib.pyplot as plt

# DTMF "1" frequencies
f_low = 697
f_high = 1209

# Sampling parameters
fs = 44100
duration = 0.01  # 10 ms for clean visualization
t = np.linspace(0, duration, int(fs * duration), endpoint=False)

# Generate tones
low_tone = np.sin(2 * np.pi * f_low * t)
high_tone = np.sin(2 * np.pi * f_high * t)

# Sum (DTMF signal)
dtmf_signal = low_tone + high_tone

# Plot
plt.figure(figsize=(12, 8))
plt.suptitle("DTMF Tone Generation for Digit '1' (697 Hz + 1209 Hz)")

# Low frequency
plt.subplot(3, 1, 1)
plt.plot(t, low_tone, label=f"{f_low} Hz", color="tab:blue")
plt.title("Low Frequency Component (697 Hz)")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.grid(True)

# High frequency
plt.subplot(3, 1, 2)
plt.plot(t, high_tone, label=f"{f_high} Hz", color="tab:orange")
plt.title("High Frequency Component (1209 Hz)")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.grid(True)

# Sum = DTMF tone
plt.subplot(3, 1, 3)
plt.plot(t, dtmf_signal, label="DTMF 1 = 697 Hz + 1209 Hz", color="tab:green")
plt.title("DTMF Waveform for Digit '1'")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.grid(True)

plt.tight_layout()
plt.show()
"""



import matplotlib.pyplot as plt
from SignalProc.Goertzel import GoertzelAlgorithm

def plot_goertzel_power(powers, title="Goertzel Power per Frequency"):
    """
    Plots the Goertzel power result as a simple bar plot.
    'powers' must be a {frequency: power} dictionary.
    """

    freqs = list(powers.keys())
    values = list(powers.values())

    plt.figure(figsize=(8, 3))  # same style size as typical time/fft plots
    plt.bar(freqs, values, width=40, edgecolor="black")

    plt.title(title)
    plt.xlabel("Frequency [Hz]")
    plt.ylabel("Power")

    plt.grid(axis="y", linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.show()


# -------------------------------------------
# 1) CREATE YOUR DTMF SIGNAL HERE
# -------------------------------------------
import numpy as np

fs = 44100
duration = 0.01
t = np.linspace(0, duration, int(fs*duration), endpoint=False)

# Example for DTMF digit "1"
f_low = 697
f_high = 1209
dtmf_signal = np.sin(2*np.pi*f_low*t) + np.sin(2*np.pi*f_high*t)


# -------------------------------------------
# 2) NOW PUT THE GOERTZEL BLOCK RIGHT HERE
# -------------------------------------------
# RUN GOERTZEL ON DTMF SIGNAL
target_freqs = [697, 770, 852, 941, 1209, 1336, 1477, 1633]

N = 205  # Goertzel block size
block = dtmf_signal[:N]   # Take first N samples

g = GoertzelAlgorithm.GoertzelAlgorithm(fs=fs, block=N, target_freqs=target_freqs)
powers = g.process(block)

# Plot the Goertzel result
plot_goertzel_power(powers)