# Class for plotting different stages of the signals
import matplotlib.pyplot as plt
from scipy.signal import sosfreqz
import numpy as np
import ast
import re


class Plotting:
    def __init__(self):
        pass

    @staticmethod
    def convert_linear_to_db(signal, ref=1.0, amin=1e-20):
        signal = np.maximum(np.abs(signal), amin)
        return 20 * np.log10(signal / ref)


    # Plot start signal in dB scale
    @staticmethod
    def plot_start_signal_db(signal_db, title="Start Signal in dB"):
        plt.figure(figsize=(10, 4))
        plt.plot(signal_db)
        plt.title(title)
        plt.xlabel("Sample Index")
        plt.ylabel("Amplitude (dB)")
        plt.grid(True)
        plt.show()
        
    @staticmethod
    def save_block_txt(seg, seg_f, bi, t_ms, lf, hf, E_low, E_high,
                    l_abs_db, l2_db, h_abs_db, h2_db,
                    blk_db, snr_low_db, snr_high_db,
                    l_dom_db, h_dom_db, twist,
                    sep_ok, abs_ok, twist_ok, dom_ok, snr_ok,
                    good, LUT, filename="first_detected_block.txt"):

        # Prepare a dictionary with all relevant info
        block_info = {
            "Seg (Used for plotting original block signal)": seg,
            "Seg_f (Used for plotting block signal after bandpass filter and window have been applied)": seg_f,
            "block_index": bi,
            "timestamp_ms": t_ms,
            "lf": lf,
            "hf": hf,
            "E_low": E_low,
            "E_high": E_high,
            "l_abs_db": l_abs_db,
            "l2_db": l2_db,
            "h_abs_db": h_abs_db,
            "h2_db": h2_db,
            "blk_db": blk_db,
            "snr_low_db": snr_low_db,
            "snr_high_db": snr_high_db,
            "l_dom_db": l_dom_db,
            "h_dom_db": h_dom_db,
            "twist_db": twist,
            "sep_ok": sep_ok,
            "abs_ok": abs_ok,
            "twist_ok": twist_ok,
            "dom_ok": dom_ok,
            "snr_ok": snr_ok,
            "good": good,
            "symbol_detected": LUT.get((lf, hf), "?")
        }

        # Save to text file
        with open(filename, "w") as f:
            f.write("=== First Detected DTMF Block Info ===\n\n")
            for key, value in block_info.items():
                f.write(f"{key}: {value}\n")

        print(f"[INFO] Saved first detected DTMF block info to '{filename}'")

    @staticmethod
    def open_txt_file_return_seg_segf(filename="first_detected_block.txt"):
        seg = []
        seg_f = []
        with open(filename, "r") as f:
            content = f.read()

            seg_str = None
            segf_str = None

            for line in content.splitlines():
                line = line.strip()
                if line.startswith("Seg (Used for plotting original block signal)"):
                    seg_str = line.split(":", 1)[1].strip()
                elif line.startswith("Seg_f (Used for plotting block signal after bandpass filter and window have been applied)"):
                    segf_str = line.split(":", 1)[1].strip()

            # Function to parse a bracketed, space-separated list of numbers
            def parse_array_from_str(s):
                if not s:
                    return np.array([])
                s = s.strip("[]")  # remove brackets
                # split by whitespace or comma
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", s)
                return np.array([float(x) for x in numbers])

            seg = parse_array_from_str(seg_str)
            seg_f = parse_array_from_str(segf_str)

        return seg, seg_f

    # This function opens txt file, reads seg, and seg_f arrays, and plots them (time domain)
    @staticmethod
    def plot_saved_block_tdomain(seg=None, seg_f=None):
        plt.figure(figsize=(10,4))
        plt.plot(seg, label="Original block", alpha=0.7)
        plt.plot(seg_f, label="Filtered + windowed", alpha=0.7)
        plt.title("First detected DTMF block (time domain)")
        plt.xlabel("Sample index")
        plt.ylabel("Amplitude")
        plt.legend()
        plt.grid(True)
        plt.show()
        
    # This function opens txt file, reads seg, and seg_f arrays, and plots them (freq domain)
    @staticmethod
    def plot_saved_block_fdomain(seg=None, seg_f=None):
        plt.figure(figsize=(10,4))
        plt.magnitude_spectrum(seg, Fs=8000, scale='dB', label="Before")
        plt.magnitude_spectrum(seg_f, Fs=8000, scale='dB', label="After")
        plt.title("Frequency spectrum before vs after bandpass")
        plt.legend()
        plt.grid(True)
        plt.show()
    
        