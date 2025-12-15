import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

# ------------------------------------------------------------
# Path setup
# ------------------------------------------------------------
current_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(project_root)

from SignalProc.Plotting import Plotting

plotter = Plotting()

# ============================================================
#   READ TXT FILE (MATCHES dtmf_detector.py EXACTLY)
# ============================================================

def read_plotting_txt(filename):

    amplitudes = []
    block_symbols = []

    SNR_values = []
    sep_db_values = []
    twist_values = []

    twist_pos_thresholds = []
    twist_neg_thresholds = []

    sep_thresh_values = []
    snr_thresh_values = []
    
    RMS_values = []
    tone_flags = []

    section = None

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Reset section on headers
            if line.startswith("---"):
                section = None
                continue

            # Section headers
            if "Amplitude samples" in line:
                section = "amp"; continue
            if "Detected symbols per block" in line:
                section = "sym"; continue
            if "SNR values" in line and "threshold" not in line:
                section = "snr"; continue
            if "sep_db values" in line:
                section = "sep"; continue
            if "twist values" in line:
                section = "twist"; continue

            if "twist_pos_thresholds" in line:
                section = "twist_pos"; continue
            if "twist_neg_thresholds" in line:
                section = "twist_neg"; continue
            if "sep_thresh_values" in line:
                section = "sep_thresh"; continue
            if "snr_thresh_values" in line:
                section = "snr_thresh"; continue
                
            if "RMS_values" in line:
                section = "RMS"; continue
            if "tone_flags" in line:
                section = "tone_flags"; continue

            # Stop if name:value metadata
            if ":" in line and not line.replace('.', '', 1).isdigit():
                section = None
                continue

            try:
                if section == "amp":
                    amplitudes.append(float(line))
                elif section == "sym":
                    block_symbols.append(line)
                elif section == "snr":
                    SNR_values.append(float(line))
                elif section == "sep":
                    sep_db_values.append(float(line))
                elif section == "twist":
                    twist_values.append(float(line))
                elif section == "twist_pos":
                    twist_pos_thresholds.append(float(line))
                elif section == "twist_neg":
                    twist_neg_thresholds.append(float(line))
                elif section == "sep_thresh":
                    sep_thresh_values.append(float(line))
                elif section == "snr_thresh":
                    snr_thresh_values.append(float(line))
                elif section == "RMS":
                    RMS_values.append(float(line))
                elif section == "tone_flags":
                    tone_flags.append(int(line))
            except ValueError:
                continue

    return (
        amplitudes,
        block_symbols,
        SNR_values,
        sep_db_values,
        twist_values,
        twist_pos_thresholds,
        twist_neg_thresholds,
        sep_thresh_values,
        snr_thresh_values,
        RMS_values,
        tone_flags
    )

# ============================================================
#   PLOT + SAVE
# ============================================================

def plot_allplots_andsave_to_folder(
        src_filename,
        amplitudes,
        block_symbols,
        SNR_values,
        sep_db_values,
        twist_values,
        block_ms,
        snr_thresh,
        sep_thresh,
        twist_neg_thresholds,
        twist_pos_thresholds,
        fs=44100):

    amplitudes_arr = np.asarray(amplitudes, dtype=np.float32)

    amplitude_plot = plotter.plot_signal_amplitude(amplitudes_arr, fs)
    barplotDTMF = plotter.barplot_of_DTMFtones(block_symbols)

    barplotSNR = plotter.barplot_of_threshold(SNR_values, snr_thresh)
    barplotSep = plotter.barplot_of_threshold(sep_db_values, sep_thresh)

    barplotTwist = plotter.barplot_of_twist(
        twist_values,
        twist_neg_thresholds,
        twist_pos_thresholds
    )

    plt.figure(figsize=(18, 22))

    plt.subplot(4, 1, 1)
    plotter.plot_amplitude_and_DTMFtones(
        barplotDTMF,
        amplitude_plot,
        block_ms=block_ms
    )

    plt.subplot(4, 1, 2)
    plotter.plot_amplitude_and_thresholds(
        amplitude_plot,
        barplotSNR,
        "Adaptive SNR Threshold",
        "orange",
        block_ms=block_ms
    )

    plt.subplot(4, 1, 3)
    plotter.plot_amplitude_and_thresholds(
        amplitude_plot,
        barplotSep,
        "Adaptive Separation Threshold",
        "green",
        block_ms=block_ms
    )

    plt.subplot(4, 1, 4)
    plotter.plot_amplitude_and_twist(
        amplitude_plot,
        barplotTwist,
        block_ms=block_ms
    )

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # Save location
    signalproc_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    save_folder = os.path.join(signalproc_dir, "SignalProc", "Audio_plots")
    os.makedirs(save_folder, exist_ok=True)

    base = os.path.splitext(os.path.basename(src_filename))[0]
    save_path = os.path.join(save_folder, f"plot_{base}.png")

    plt.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close()

    print(f"Saved plot: {save_path}")

# ============================================================
#   MAIN — PROCESS ALL NEW TXT FILES
# ============================================================

if __name__ == "__main__":

    signalproc_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    txt_folder = os.path.join(signalproc_dir, "SignalProc", "Audio_plotting_txtfiles")

    plotted_log = os.path.join(txt_folder, "plotted_list.txt")

    already_plotted = set()
    if os.path.exists(plotted_log):
        with open(plotted_log, "r") as f:
            already_plotted = {line.strip() for line in f}

    for filename in sorted(os.listdir(txt_folder)):
        if not filename.endswith(".txt"):
            continue
        if filename == "plotted_list.txt":
            continue
        if filename in already_plotted:
            print(f"Skipping: {filename}")
            continue

        full_path = os.path.join(txt_folder, filename)
        print(f"\nPlotting: {filename}")

        (
            amplitudes,
            block_symbols,
            SNR_values,
            sep_db_values,
            twist_values,
            twist_pos_thresholds,
            twist_neg_thresholds,
            sep_thresh_values,
            snr_thresh_values,
            RMS_values,
            tone_flags
        ) = read_plotting_txt(full_path)


        plot_allplots_andsave_to_folder(
            full_path,
            amplitudes,
            block_symbols,
            SNR_values,
            sep_db_values,
            twist_values,
            block_ms=40,
            snr_thresh=snr_thresh_values,
            sep_thresh=sep_thresh_values,
            twist_neg_thresholds=twist_neg_thresholds,
            twist_pos_thresholds=twist_pos_thresholds,
            fs=44100
        )

        with open(plotted_log, "a") as f:
            f.write(filename + "\n")

    print("\n✅ Done — all new files plotted.")
