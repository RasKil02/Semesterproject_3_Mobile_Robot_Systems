import os
import numpy as np
from statistics import mean

# -----------------------------
#   READ A SINGLE DEBUG FILE
# -----------------------------
def read_debug_file(path):

    sections = {
        "amplitudes": [],
        "symbols": [],
        "snr_values": [],
        "sep_db": [],
        "twist_values": [],
        "twist_pos_thresholds": [],
        "twist_neg_thresholds": [],
        "sep_thresh_values": [],
        "snr_thresh_values": [],
        "RMS_values": [],
        "tone_flags": []
    }

    section = None

    with open(path, "r") as f:
        for line in f:
            line = line.strip()

            if not line:
                continue

            # Section headers
            if line.startswith("Amplitude samples"):
                section = "amplitudes"; continue
            if line.startswith("Detected symbols per block"):
                section = "symbols"; continue
            if line.startswith("SNR values"):
                section = "snr_values"; continue
            if line.startswith("sep_db values"):
                section = "sep_db"; continue
            if line.startswith("twist values"):
                section = "twist_values"; continue
            if line.startswith("twist_pos_thresholds"):
                section = "twist_pos_thresholds"; continue
            if line.startswith("twist_neg_thresholds"):
                section = "twist_neg_thresholds"; continue
            if line.startswith("sep_thresh_values"):
                section = "sep_thresh_values"; continue
            if line.startswith("snr_thresh_values"):
                section = "snr_thresh_values"; continue
            if line.startswith("RMS_values"):
                section = "RMS_values"; continue
            if line.startswith("tone_flags"):
                section = "tone_flags"; continue

            # Stop if metadata like: "Detected digits: *#..."
            if ":" in line and not line.replace('.', '', 1).isdigit():
                section = None
                continue

            # Parse numeric or symbol values
            try:
                if section in ["amplitudes", "snr_values", "sep_db",
                               "twist_values", "twist_pos_thresholds",
                               "twist_neg_thresholds", "sep_thresh_values",
                               "snr_thresh_values", "RMS_values"]:
                    sections[section].append(float(line))

                elif section == "tone_flags":
                    sections[section].append(int(line))

                elif section == "symbols":
                    sections[section].append(line)

            except ValueError:
                continue

    return sections


# ----------------------------------------------------------
#   ANALYZE FILE — split into tone vs no-tone stats
# ----------------------------------------------------------
def analyze_file(sections):

    tone = np.array(sections["tone_flags"]) == 1

    rms_tone = np.array(sections["RMS_values"])[tone]
    rms_noise = np.array(sections["RMS_values"])[~tone]

    snr = np.array(sections["snr_values"])
    sep = np.array(sections["sep_db"])
    twist = np.array(sections["twist_values"])

    results = {
        "mean_rms_tone": float(rms_tone.mean()) if len(rms_tone) > 0 else None,
        "mean_rms_noise": float(rms_noise.mean()) if len(rms_noise) > 0 else None,

        "mean_snr_tone": float(snr[tone].mean()) if any(tone) else None,
        "mean_snr_noise": float(snr[~tone].mean()) if any(~tone) else None,

        "mean_sep_tone": float(sep[tone].mean()) if any(tone) else None,
        "mean_sep_noise": float(sep[~tone].mean()) if any(~tone) else None,

        "mean_twist_tone": float(twist[tone].mean()) if any(tone) else None,
        "mean_twist_noise": float(twist[~tone].mean()) if any(~tone) else None,
    }

    return results


# ----------------------------------------------------------
#   MAIN FUNCTION: read all log files in the folder
# ----------------------------------------------------------
def analyze_all_files(folder):

    print("\n=== ANALYZING ALL DTMF LOG FILES ===\n")

    all_results = {}

    for filename in os.listdir(folder):
        if not filename.startswith("dtmf_debug"):
            continue
        if not filename.endswith(".txt"):
            continue

        full_path = os.path.join(folder, filename)
        print(f"Processing {filename}")

        sections = read_debug_file(full_path)
        stats = analyze_file(sections)

        all_results[filename] = stats

    return all_results


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    folder = os.path.join(base_dir, "..", "SignalProc", "Audio_plotting_txtfiles")
    folder = os.path.abspath(folder)


    results = analyze_all_files(folder)

    print("\n=== SUMMARY TABLE ===")
    for fname, stats in results.items():
        print(f"\n{fname}:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
