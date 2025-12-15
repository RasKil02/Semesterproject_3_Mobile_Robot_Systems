import os
import numpy as np

# ------------------------------------------------------------
# READ ONLY THE REAL METRIC ARRAYS (ignore thresholds)
# ------------------------------------------------------------
def read_debug_file(path):

    RMS, SNR, SEP, TWIST, FLAGS = [], [], [], [], []
    section = None

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Section switches
            if line.startswith("RMS_values"):
                section = "rms"; continue
            if line.startswith("SNR values"):
                section = "snr"; continue
            if line.startswith("sep_db values"):
                section = "sep"; continue
            if line.startswith("twist values"):
                section = "twist"; continue
            if line.startswith("tone_flags"):
                section = "flags"; continue

            # Stop reading real data if threshold arrays appear
            if line.startswith("sep_thresh_values"): section = None; continue
            if line.startswith("snr_thresh_values"): section = None; continue
            if line.startswith("twist_pos_thresholds"): section = None; continue
            if line.startswith("twist_neg_thresholds"): section = None; continue

            # Skip meta lines
            if ":" in line and not line.replace('.', '', 1).isdigit():
                continue

            # Parse based on active section
            try:
                if section == "rms":
                    RMS.append(float(line))
                elif section == "snr":
                    SNR.append(float(line))
                elif section == "sep":
                    SEP.append(float(line))
                elif section == "twist":
                    TWIST.append(float(line))
                elif section == "flags":
                    FLAGS.append(int(line))
            except:
                pass

    return (
        np.array(RMS),
        np.array(SNR),
        np.array(SEP),
        np.array(TWIST),
        np.array(FLAGS)
    )


# ------------------------------------------------------------
# COMPUTE: 10 LOWEST TONE VALUES + 10 HIGHEST NOISE VALUES
# ------------------------------------------------------------
def compute_bounds(values, flags):

    # Tone: smallest 10
    tone_vals = values[flags == 1]
    if tone_vals.size > 0:
        tone_vals_sorted = np.sort(tone_vals)
        tone_est = tone_vals_sorted[:min(10, len(tone_vals_sorted))].mean()
    else:
        tone_est = None

    # Noise: largest 10
    noise_vals = values[flags == 0]
    if noise_vals.size > 0:
        noise_vals_sorted = np.sort(noise_vals)
        noise_est = noise_vals_sorted[-min(10, len(noise_vals_sorted)):].mean()
    else:
        noise_est = None

    return tone_est, noise_est


# ------------------------------------------------------------
# GROUPING: 8 FILES PER TEST
# ------------------------------------------------------------
def determine_test_group(index):

    group_num = index // 8 + 1  # integer division
    return f"Test {group_num}"


# ------------------------------------------------------------
# PROCESS ALL FILES
# ------------------------------------------------------------
def process_folder(folder):

    results = {}  # group → list of metric dicts

    filenames = sorted([f for f in os.listdir(folder) if f.endswith(".txt")])

    for idx, fname in enumerate(filenames):
        full = os.path.join(folder, fname)
        print("Reading:", fname)

        RMS, SNR, SEP, TWIST, FLAGS = read_debug_file(full)

        # compute min tone / max noise estimates
        rms_tone_min, rms_noise_max = compute_bounds(RMS, FLAGS)
        snr_tone_min, snr_noise_max = compute_bounds(SNR, FLAGS)
        sep_tone_min, sep_noise_max = compute_bounds(SEP, FLAGS)
        twist_tone_min, twist_noise_max = compute_bounds(TWIST, FLAGS)

        group = determine_test_group(idx)

        entry = {
            "file": fname,
            "rms_tone_min": rms_tone_min,
            "rms_noise_max": rms_noise_max,
            "snr_tone_min": snr_tone_min,
            "snr_noise_max": snr_noise_max,
            "sep_tone_min": sep_tone_min,
            "sep_noise_max": sep_noise_max,
            "twist_tone_min": twist_tone_min,
            "twist_noise_max": twist_noise_max,
        }

        results.setdefault(group, []).append(entry)

    return results


# ------------------------------------------------------------
# PRINT SUMMARY PER TEST
# ------------------------------------------------------------
def print_summary(results):

    def extract(entries, key):
        vals = [e[key] for e in entries if e[key] is not None]
        if not vals:
            return None
        return min(vals), max(vals)

    for group, entries in results.items():
        print("\n==========", group, "==========")

        for field in [
            "rms_tone_min", "snr_tone_min", "sep_tone_min", "twist_tone_min",
            "rms_noise_max", "snr_noise_max", "sep_noise_max", "twist_noise_max"
        ]:
            mm = extract(entries, field)
            if mm:
                print(f"{field}: {mm[0]:.4f} – {mm[1]:.4f}")
            else:
                print(f"{field}: None")


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
if __name__ == "__main__":

    base = os.path.dirname(os.path.abspath(__file__))
    folder = os.path.join(base, "..", "SignalProc", "Audio_plotting_txtfiles")
    folder = os.path.abspath(folder)

    results = process_folder(folder)
    print_summary(results)
