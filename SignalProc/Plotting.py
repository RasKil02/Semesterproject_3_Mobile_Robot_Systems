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
        mag = np.abs(amplitude_arr)

        window_size = int(0.05 * fs)
        envelope = np.convolve(mag, np.ones(window_size)/window_size, mode='valid')

        t = np.arange(len(envelope)) / fs

        return {
            "t": t,
            "envelope": envelope
        }

    def barplot_of_DTMFtones(self, block_symbols):
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
        
    def barplot_of_threshold(self, values, threshold):
        x_values = np.arange(len(values))
        y_values = values
        
        thresholdline = [threshold] * len(values)
        
        return {
            "x": x_values,
            "y": y_values,
            "thresholdline": thresholdline
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

    def plot_amplitude_and_thresholds(self, amplitudeplot, thresholdplot, title, block_ms=40):

        # ----- Extract amplitude data -----
        t = amplitudeplot['t']
        envelope = amplitudeplot['envelope']

        # ----- Extract threshold bar plot -----
        x_blocks = thresholdplot["x"]
        y_values = thresholdplot["y"]
        threshold_line = thresholdplot["thresholdline"]

        block_sec = block_ms / 1000.0
        x_blocks_sec = x_blocks * block_sec

        # ---------- Create figure ----------
        plt.figure(figsize=(20, 6))

        # LEFT AXIS: amplitude envelope
        ax1 = plt.gca()
        ax1.plot(t, envelope, color="red", linewidth=1.4)
        ax1.set_ylabel("Amplitude", color="red")
        ax1.tick_params(axis="y", labelcolor="red")
        ax1.grid(True, linestyle="--", alpha=0.3)

        # RIGHT AXIS: threshold bars
        ax2 = ax1.twinx()

        bar_width = block_sec * 0.9

        # Bars = actual SNR/minDB/sepDB/etc values per block
        ax2.bar(
            x_blocks_sec,
            y_values,
            width=bar_width,
            color="blue",
            edgecolor="black",
            alpha=0.4,
            align="center",
            label="Measured Value"
        )

        # Threshold line
        ax2.plot(
            x_blocks_sec,
            threshold_line,
            color="yellow",
            linestyle="dashed",
            linewidth=1.5,
            label="Threshold"
        )

        ax2.set_ylabel("Threshold-measured value", color="blue")
        ax2.tick_params(axis="y", labelcolor="blue")

        ax1.set_xlabel("Time (seconds)")
        ax1.set_xlim(0, max(t.max(), x_blocks_sec.max() + block_sec))

        plt.title(title)
        plt.tight_layout()
        plt.show()





