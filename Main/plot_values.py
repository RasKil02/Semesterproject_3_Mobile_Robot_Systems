from SignalProc.Plotting import Plotting
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

plotter = Plotting()

def read_plotting_txt(filename):
    amplitudes = []
    block_symbols = []
    SNR_values = []
    sep_db_values = []
    dom_db_values = []
    twist_values = []

    # thresholds default (in case missing)
    snr_db = None
    sep_db = None
    dom_db = None
    twist_pos_db = None
    twist_neg_db = None

    section = None

    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # --- Detect threshold lines ---
            if "sep_db threshold:" in line:
                sep_db = float(line.split(":")[1])
                continue
            if "dom_db threshold:" in line:
                dom_db = float(line.split(":")[1])
                continue
            if "snr_db threshold:" in line:
                snr_db = float(line.split(":")[1])
                continue
            if "twist_pos_db:" in line:
                twist_pos_db = float(line.split(":")[1])
                continue
            if "twist_neg_db:" in line:
                twist_neg_db = float(line.split(":")[1])
                continue

            # --- Detect section headers ---
            if "Amplitude samples" in line:
                section = "amp"
                continue
            if "Detected symbols per block" in line:
                section = "sym"
                continue
            if "SNR values" in line:
                section = "snr"
                continue
            if "sep_db values" in line:
                section = "sep"
                continue
            if "dom_db values" in line:
                section = "dom"
                continue
            if "twist values" in line:
                section = "twist"
                continue

            # --- Parse section lines ---
            if section == "amp":
                amplitudes.append(float(line))
            elif section == "sym":
                block_symbols.append(line)
            elif section == "snr":
                SNR_values.append(float(line))
            elif section == "sep":
                sep_db_values.append(float(line))
            elif section == "dom":
                dom_db_values.append(float(line))
            elif section == "twist":
                twist_values.append(float(line))

    return (amplitudes, block_symbols, SNR_values, sep_db_values, dom_db_values, twist_values,
            snr_db, sep_db, dom_db, twist_neg_db, twist_pos_db)


def plot_allplots_andsave_to_folder(amplitudes, block_symbols, SNR_values, sep_db_values, dom_db_values, twist_values, block_ms,
                                    snr_db, sep_db, dom_db, twist_neg_db, twist_pos_db, fs=44100):
    amplitudes_arr = np.array(amplitudes, dtype=np.float32)
    amplitude_plot = plotter.plot_signal_amplitude(amplitudes_arr, fs)
    barplotDTMF = plotter.barplot_of_DTMFtones(block_symbols)
    barplotSNR = plotter.barplot_of_threshold(SNR_values, snr_db)
    barplotSepDB = plotter.barplot_of_threshold(sep_db_values, sep_db)
    barplotDomDB = plotter.barplot_of_threshold(dom_db_values, dom_db)
    barplotTwist = plotter.barplot_of_twist(twist_values, twist_neg_db, twist_pos_db)

    thresholdplot = plt.figure(figsize=(18, 25))  # one big figure

    plt.subplot(5, 1, 1)
    plotter.plot_amplitude_and_DTMFtones(barplotDTMF, amplitude_plot, block_ms=block_ms)

    plt.subplot(5, 1, 2)
    plotter.plot_amplitude_and_thresholds(amplitude_plot, barplotSNR, "SNR and Amplitude", "orange", block_ms=block_ms)

    plt.subplot(5, 1, 3)
    plotter.plot_amplitude_and_thresholds(amplitude_plot, barplotSepDB, "Sep_db and Amplitude", "green", block_ms=block_ms)

    plt.subplot(5, 1, 4)
    plotter.plot_amplitude_and_thresholds(amplitude_plot, barplotDomDB, "Dom_db and Amplitude", "purple", block_ms=block_ms)

    plt.subplot(5, 1, 5)
    plotter.plot_amplitude_and_twist(amplitude_plot, barplotTwist, block_ms=block_ms)

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    signalproc_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    save_folder = os.path.join(signalproc_dir, "SignalProc", "Audio_plots")
    os.makedirs(save_folder, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    save_path = os.path.join(save_folder, f"thresholdplot_{timestamp}.png")

    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"Saved plot to: {save_path}")
    
if __name__ == "__main__":
    amplitudes, block_symbols, SNR_values, sep_db_values, dom_db_values, twist_values, \
    snr_db, sep_db, dom_db, twist_neg_db, twist_pos_db = read_plotting_txt("SignalProc/Audio_plotting_txtfiles/dtmf_debug_2025-12-04_09-45-59.txt")
    plot_allplots_andsave_to_folder(amplitudes, block_symbols, SNR_values, sep_db_values, dom_db_values, twist_values, block_ms=40,
                                    snr_db=snr_db, sep_db=sep_db, dom_db=dom_db, twist_neg_db=twist_neg_db, twist_pos_db=twist_pos_db, fs=44100)