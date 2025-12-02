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
        
    def barplot_of_twist(self, twist_values, twist_neg, twist_pos):
        x_values = np.arange(len(twist_values))
        y_values = twist_values

        # two threshold lines
        neg_line = np.full(len(twist_values), twist_neg)
        pos_line = np.full(len(twist_values), twist_pos)

        return {
            "x": x_values,
            "y": y_values,
            "neg_line": neg_line,
            "pos_line": pos_line
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
        plt.figure(figsize=(18, 6))

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
            alpha=0.6,
            align="center"
        )

        ax2.set_ylabel("DTMF Symbol", color="black")
        ax2.set_yticks(range(len(dtmf_chars)))
        ax2.set_yticklabels(dtmf_chars)
        
        for y in range(len(dtmf_chars)):
            ax2.axhline(
                y,
                color="black",
                linestyle="-",
                linewidth=0.6,
                alpha=0.7)

        # X axis
        ax1.set_xlabel("Time (seconds)")
        ax1.set_xlim(0, max(t.max(), x_blocks_sec.max() + block_sec))

        plt.title("Amplitude Envelope + DTMF Detection (Overlayed)")
        plt.tight_layout()

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
        plt.figure(figsize=(18, 6))

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
            alpha=0.6,
            align="center",
            label="Measured Value"
        )

        # Threshold line
        ax2.plot(
            x_blocks_sec,
            threshold_line,
            color="yellow",
            linestyle="dashed",
            linewidth=2,
            label="Threshold"
        )

        ax2.set_ylabel("Threshold-measured value", color="blue")
        ax2.tick_params(axis="y", labelcolor="blue")

        Rmin, Rmax = ax2.get_ylim()
        if Rmin < 0:
        # extend left axis just enough to align visually
            ax1.set_ylim(bottom = Rmin * 0.002)   # small proportional extension

        ax1.set_xlabel("Time (seconds)")
        ax1.set_xlim(0, max(t.max(), x_blocks_sec.max() + block_sec))

        plt.title(title)
        plt.tight_layout()

    def plot_amplitude_and_twist(self, amplitudeplot, twistplot, block_ms=40):

        t = amplitudeplot["t"]
        envelope = amplitudeplot["envelope"]

        x_blocks = twistplot["x"]
        y_values = twistplot["y"]
        neg_line = twistplot["neg_line"]
        pos_line = twistplot["pos_line"]

        block_sec = block_ms / 1000.0
        x_blocks_sec = x_blocks * block_sec

        plt.figure(figsize=(20,6))

        # LEFT AXIS: amplitude
        ax1 = plt.gca()
        ax1.plot(t, envelope, color="red", linewidth=1.4)
        ax1.set_ylabel("Amplitude", color="red")
        ax1.tick_params(axis="y", labelcolor="red")
        ax1.grid(True, linestyle="--", alpha=0.3)

        # RIGHT AXIS: twist bars
        ax2 = ax1.twinx()
        ax2.bar(
            x_blocks_sec, y_values,
            width=block_sec * 0.9,
            color="blue", edgecolor="black",
            alpha=0.6
        )

        # Twist threshold lines
        ax2.plot(x_blocks_sec, neg_line, color="yellow",
                linestyle="dashed", linewidth=1.4)
        ax2.plot(x_blocks_sec, pos_line, color="yellow",
                linestyle="dashed", linewidth=1.4)

        ax2.set_ylabel("Twist (dB)", color="blue")
        ax2.tick_params(axis="y", labelcolor="blue")

        Rmin, Rmax = ax2.get_ylim()      # twist axis limits

        if Rmin < 0:
            # How much of twist-range lies below zero?
            frac = abs(Rmin) / (Rmax - Rmin)

            # Get amplitude axis limits
            Lmin, Lmax = ax1.get_ylim()
            Lrange = Lmax - Lmin

            # Extend amplitude axis downward by equivalent fraction
            new_Lmin = -frac * Lrange / (1 - frac)

            ax1.set_ylim(new_Lmin, Lmax)
        # ------------------------------------------------------------

        ax1.set_xlabel("Time (seconds)")
        ax1.set_xlim(0, max(t.max(), x_blocks_sec.max() + block_sec))

        plt.title("Twist and Amplitude Plot")
        plt.tight_layout()

    def plot_amplitude_and_all_thresholds(
            self,
            amplitudeplot,
            DTMFtoneplot,
            snrplot,
            sepdpplot,
            domdbplot,
            twistplot,
            block_ms=40):

        t = amplitudeplot["t"]
        envelope = amplitudeplot["envelope"]
        block_sec = block_ms / 1000.0

        # ------------------------------------------------------------
        # Create figure with 2 stacked subplots
        # ------------------------------------------------------------
        fig, (ax_top, ax_bottom) = plt.subplots(
            2, 1,
            figsize=(20, 12),
            sharex=True,
            gridspec_kw={'height_ratios': [1, 1.3]}
        )

        # ============================================================
        #  TOP PLOT: Amplitude + DTMF tones
        # ============================================================

        # 1) DTMF bars first (so amplitude draws on top)
        x_dtmf = DTMFtoneplot["x"] * block_sec
        y_dtmf = DTMFtoneplot["y"]

        dtmf_chars = ["0","1","2","3","4","5","6","7","8","9","A","B","C","D","*","#"]

        ax2_top = ax_top.twinx()
        bar_width = block_sec * 0.9
        ax2_top.bar(
            x_dtmf, y_dtmf,
            width=bar_width,
            color="cornflowerblue",
            edgecolor="black",
            alpha=0.35
        )
        ax2_top.set_ylabel("DTMF Tone", color="blue")
        ax2_top.set_yticks(range(len(dtmf_chars)))
        ax2_top.set_yticklabels(dtmf_chars)
        ax2_top.tick_params(axis="y", labelcolor="blue")

        # 2) Amplitude curve on top
        ax_top.plot(t, envelope, color="red", linewidth=1.6)
        ax_top.set_ylabel("Amplitude", color="red")
        ax_top.tick_params(axis="y", labelcolor="red")
        ax_top.grid(True, linestyle="--", alpha=0.3)
        ax_top.set_title("Amplitude + DTMF Detection")

        # ============================================================
        #  BOTTOM PLOT: Amplitude + ALL THRESHOLDS
        # ============================================================

        # Always draw amplitude FIRST
        ax_bottom.plot(t, envelope, color="red", linewidth=1.4)
        ax_bottom.set_ylabel("Amplitude", color="red")
        ax_bottom.tick_params(axis="y", labelcolor="red")
        ax_bottom.grid(True, linestyle="--", alpha=0.3)

        # Threshold plot definitions
        plots = [
            (snrplot,   "SNR",          "blue"),
            (sepdpplot, "Separation dB","green"),
            (domdbplot, "Dominant dB",  "purple"),
            (twistplot, "Twist (dB)",   "cyan"),
        ]

        # Distance between right-side axes
        offset = 0

        for plot_data, label, color in plots:

            x_blocks = plot_data["x"] * block_sec
            y_values = plot_data["y"]

            th_line  = plot_data.get("thresholdline", None)
            neg_line = plot_data.get("neg_line", None)
            pos_line = plot_data.get("pos_line", None)

            # Create new axis on right, spaced outward
            ax_new = ax_bottom.twinx()
            ax_new.spines["right"].set_position(("outward", 70 + offset))
            offset += 70

            # Bars
            ax_new.bar(
                x_blocks, y_values,
                width=block_sec * 0.9,
                color=color,
                edgecolor="black",
                alpha=0.28
            )

            # Thin dashed threshold lines
            if th_line is not None:
                ax_new.plot(x_blocks, th_line, color=color,
                            linestyle="--", linewidth=1.0)

            if neg_line is not None:
                ax_new.plot(x_blocks, neg_line, color=color,
                            linestyle="--", linewidth=1.0)

            if pos_line is not None:
                ax_new.plot(x_blocks, pos_line, color=color,
                            linestyle="--", linewidth=1.0)

            # Label axis
            ax_new.set_ylabel(label, color=color)
            ax_new.tick_params(axis="y", labelcolor=color)

        # ------------------------------------------------------------
        ax_bottom.set_xlabel("Time (seconds)")
        ax_bottom.set_xlim(0, t.max())
        ax_bottom.set_title("Amplitude + SNR + Separation dB + Dominant dB + Twist dB")

        plt.tight_layout()

