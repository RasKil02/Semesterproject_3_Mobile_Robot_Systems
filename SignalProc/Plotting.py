# Class for plotting different stages of the signals
import matplotlib.pyplot as plt
from scipy.signal import sosfreqz
import numpy as np
import ast
import re


class Plotting:
    def __init__(self):
        pass
            
    def plot_signal_amplitude(self, amplitude_arr, fs=44100):
        print(amplitude_arr)
        lengthoflist = len(amplitude_arr)
        print("Length of list (amplitude):", lengthoflist)
        
        trim_ms = 2000
        cut = int((trim_ms / 1000) * fs)
        amplitude_arr = amplitude_arr[cut:]

        mag = np.abs(amplitude_arr)

        window_size = int(0.05 * fs)
        envelope = np.convolve(mag, np.ones(window_size)/window_size, mode='valid')

        t = np.arange(len(envelope)) / fs

        return {
            "t": t,
            "envelope": envelope
        }

    def barplot_of_DTMFtones(self, block_symbols):
        print("Received block symbols (symbols):", block_symbols)
        lengthoflist = len(block_symbols)
        print("Length of list:", lengthoflist)

        print("Block Symbols:", block_symbols)

        dtmf_chars = [""] + ["0","1","2","3","4","5","6","7","8","9",
                            "A","B","C","D","*","#"]

        char_to_y = {ch: i for i, ch in enumerate(dtmf_chars)}

        y_values = [char_to_y.get(ch, 0) for ch in block_symbols]
        x_values = np.arange(len(block_symbols))

        return {
            "x": x_values,
            "y": y_values,
            "dtmf_chars": dtmf_chars
        }


    def plot_amplitude_and_DTMFtones(self, barplot, amplitudeplot, block_ms=40):

        # ----- Extract amplitude data -----
        t = amplitudeplot['t']              # seconds
        envelope = amplitudeplot['envelope']

        # ----- Extract DTMF bar data -----
        x_blocks = barplot['x']             # block indices
        y_blocks = barplot['y']             # categorical ints
        dtmf_chars = barplot['dtmf_chars']  # labels

        # Convert block index → seconds
        block_sec = block_ms / 1000.0
        x_blocks_sec = x_blocks * block_sec

        # ---------- FIX 1: compensate for amplitude trimming ----------
        TRIM_SEC = 2.0  # because you trimmed first 2000 ms = 2 seconds
        x_blocks_sec = x_blocks_sec - TRIM_SEC

        # ---------- FIX 2: compensate for envelope smoothing delay ----------
        # envelope uses 50ms window, so shift bars left by half → 25 ms
        ALIGN_SHIFT = 0.025  # 25 ms
        x_blocks_sec = x_blocks_sec - ALIGN_SHIFT

        # Remove bars that fall before t=0 after shifting
        valid = x_blocks_sec >= 0
        x_blocks_sec = x_blocks_sec[valid]
        y_blocks = np.array(y_blocks)[valid]

        # ---------- Create the combined plot ----------
        plt.figure(figsize=(20, 6))

        # LEFT AXIS: amplitude envelope
        ax1 = plt.gca()
        ax1.plot(t, envelope, color="red", linewidth=1.8)
        ax1.set_ylabel("Amplitude", color="red")
        ax1.tick_params(axis="y", labelcolor="red")
        ax1.set_ylim(0, 1.1)
        ax1.grid(True, linestyle="--", alpha=0.3)

        # RIGHT AXIS: DTMF symbol bars
        ax2 = ax1.twinx()

        bar_width = block_sec * 0.9   # bars nearly fill block width

        ax2.bar(
            x_blocks_sec,
            y_blocks,
            width=bar_width,
            color="blue",
            edgecolor="black",
            alpha=0.5,
            align="center"
        )

        ax2.set_ylabel("DTMF Symbol", color="black")
        ax2.set_yticks(range(len(dtmf_chars)))
        ax2.set_yticklabels(dtmf_chars)

        # X axis
        ax1.set_xlabel("Time (seconds)")
        ax1.set_xlim(0, max(t.max(), x_blocks_sec.max() + block_sec))

        plt.title("Amplitude Envelope + DTMF Detection (Overlayed)")
        plt.tight_layout()
        plt.show()



