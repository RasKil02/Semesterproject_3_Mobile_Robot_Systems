# dtmf_detector.py
import numpy as np
import argparse
import os

import matplotlib
matplotlib.use("Agg")

from SignalProc.BandpassFilter import BandPassFilter
from SignalProc.Goertzel import GoertzelAlgorithm
from SignalProc.AudioSampling import AudioSampler
from SignalProc.Plotting import Plotting
import matplotlib.pyplot as plt
from datetime import datetime

# --- DTMF opsætning ---
FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = {
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B',
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

plotter = Plotting()

# Sets up hanning window
def hann(N: int):
    n = np.arange(N)
    return 0.5 * (1 - np.cos(2*np.pi*n/(N-1)))

EPS = 1e-12
def db10(x: float) -> float:
    return 10*np.log10(max(float(x), EPS))

class DigitStabilizer:
    """
    Enkel tids-baseret stabilisering:
      - hold_ms: minimum varighed en kandidat skal være stabil for at emitte
      - miss_ms: max hul (mistet detektion) uden at miste låsen
      - gap_ms : krævet stilhed før næste symbol (hindrer dobbeltskud)
    Tilstande: IDLE -> CANDIDATE -> LOCKED -> GAP
    """
    def __init__(self, hold_ms=130, miss_ms=70, gap_ms=78):
        self.hold_ms = float(hold_ms)
        self.miss_ms = float(miss_ms)
        self.gap_ms  = float(gap_ms)

        self.state = "IDLE"      # IDLE | CANDIDATE | LOCKED | GAP
        self.curr = None         # kandidat / låst symbol
        self.last_locked = None  # senest emitterede symbol
        self.t_start = None      # starttid for kandidat
        self.t_last_seen = None  # sidste gang vi så curr (til miss_ms)
        self.gap_start = None    # starttid for GAP

    def update(self, sym: str, now_ms: float):
        # Normalisér
        if sym not in "1234567890ABCD*#":
            sym = "?"

        # --- IDLE ---
        if self.state == "IDLE":
            if sym == "?":
                return None
            self.state = "CANDIDATE"
            self.curr = sym
            self.t_start = now_ms
            self.t_last_seen = now_ms
            return None

        # --- CANDIDATE ---
        if self.state == "CANDIDATE":
            if sym == self.curr:
                self.t_last_seen = now_ms
                if (now_ms - self.t_start) >= self.hold_ms:
                    # lås og emit
                    self.state = "LOCKED"
                    self.last_locked = self.curr
                    return self.curr
                return None
            elif sym == "?":
                # mistet før lås -> reset
                self._reset_to_idle()
                return None
            else:
                # ny kandidat
                self.curr = sym
                self.t_start = now_ms
                self.t_last_seen = now_ms
                return None

        # --- LOCKED ---
        if self.state == "LOCKED":
            if sym == self.curr:
                self.t_last_seen = now_ms
                return None
            elif sym == "?":
                if (now_ms - self.t_last_seen) > self.miss_ms:
                    # slip lås og kræv stilhed før ny
                    self.state = "GAP"
                    self.gap_start = now_ms
                return None
            else:
                # andet symbol uden stilhed -> ny kandidat
                self.state = "CANDIDATE"
                self.curr = sym
                self.t_start = now_ms
                self.t_last_seen = now_ms
                return None

        # --- GAP ---
        if self.state == "GAP":
            if sym == "?":
                if self.gap_start is None:
                    self.gap_start = now_ms
                if (now_ms - self.gap_start) >= self.gap_ms:
                    # klar igen
                    self._reset_to_idle()
                return None
            else:
                # Hvis samme som sidst låste: ignorér indtil stilhed
                if self.last_locked is not None and sym == self.last_locked:
                    self.t_last_seen = now_ms
                    return None
                # ellers start ny kandidat med det samme
                self.state = "CANDIDATE"
                self.curr = sym
                self.t_start = now_ms
                self.t_last_seen = now_ms
                return None

        return None  # default

    def _reset_to_idle(self):
        self.state = "IDLE"
        self.curr = None
        self.t_start = None
        self.t_last_seen = None
        self.gap_start = None

class DTMFDetector:
    def __init__(self,
                 # Audio input
                 fs: int = 44100,
                 block_ms: float = 40.0,
                 lowcut: float   = 620.0,
                 highcut: float  = 1700.0,
                 bp_order: int   = 4,

                 # Absolute threshold
                 twist_pos_db: float = +6.0,
                 twist_neg_db: float = -6.0,

                 # --- Adaptive separation thresholds ---
                 sep_min: float = 13,
                 sep_max: float = 25,

                 # --- Adaptive SNR thresholds ---
                 snr_min: float = 11,
                 snr_max: float = 25,

                 # --- Adaptive twist thresholds ---
                 twist_pos_default: float = +6.0,
                 twist_neg_default: float = -6.0,
                 twist_pos_max: float = +30.0,
                 twist_neg_min: float = -30.0,

                 # RMS range for adaptivity
                 rms_min: float = 0.1,
                 rms_max: float = 0.25):
        
        # Sampling configuration
        self.fs = int(fs)
        self.block = max(1, int(self.fs * (block_ms / 1000.0)))

        # Filters and window
        self.bp  = BandPassFilter(self.fs, lowcut, highcut, bp_order)
        self.win = hann(self.block)

        # Goertzel analyzers
        self.g_low  = GoertzelAlgorithm(self.fs, self.block, FREQS_LOW)
        self.g_high = GoertzelAlgorithm(self.fs, self.block, FREQS_HIGH)

        # Absolute threshold
        self.twist_pos_db = float(twist_pos_db)
        self.twist_neg_db = float(twist_neg_db)

        # Adaptive separation
        self.sep_min = float(sep_min)
        self.sep_max = float(sep_max)

        # Adaptive SNR
        self.snr_min = float(snr_min)
        self.snr_max = float(snr_max)

        # Adaptive twist parameters
        self.twist_pos_default = float(twist_pos_default)
        self.twist_neg_default = float(twist_neg_default)
        self.twist_pos_max     = float(twist_pos_max)
        self.twist_neg_min     = float(twist_neg_min)

        # RMS adaptivity range
        self.rms_min = float(rms_min)
        self.rms_max = float(rms_max)

    def save_plotting_txt(self, digits, amplitudes, block_symbols, SNR_values,
                        sep_db_values, twist_values, twist_neg_values, twist_pos_values, 
                        sep_thresh_values, snr_thresh_values, RMS_values, tone_flags):

        try:
            # Base directory = directory of THIS file
            base_dir = os.path.dirname(os.path.abspath(__file__))

            # Create / find the subfolder
            save_folder = os.path.join(base_dir, "Audio_plotting_txtfiles")
            os.makedirs(save_folder, exist_ok=True)

            # Create filename with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = os.path.join(save_folder, f"dtmf_debug_{timestamp}.txt")

            with open(filename, "w") as f:

                f.write("=== DTMF DEBUG DATA ===\n")
                f.write(f"Detected digits: {''.join(digits)}\n")

                f.write("\n--- Threshold Settings ---\n")
                f.write(f"twist_pos_db: {self.twist_pos_db}\n")
                f.write(f"twist_neg_db: {self.twist_neg_db}\n")
                f.write(f"sep_thresh_values: {sep_thresh_values}\n")  
                f.write(f"snr_thresh_values: {snr_thresh_values}\n")
                f.write("\n--- Collected Metrics ---\n")

                f.write("\nAmplitude samples:\n")
                for a in amplitudes:
                    f.write(f"{a}\n")

                f.write("\nDetected symbols per block:\n")
                for s in block_symbols:
                    f.write(f"{s}\n")

                f.write("\nSNR values:\n")
                for v in SNR_values:
                    f.write(f"{v}\n")

                f.write("\nsep_db values:\n")
                for v in sep_db_values:
                    f.write(f"{v}\n")

                f.write("\ntwist values:\n")
                for v in twist_values:
                    f.write(f"{v}\n")
                
                f.write("\ntwist_pos_thresholds:\n")
                for v in twist_pos_values:
                    f.write(f"{v}\n")

                f.write("\ntwist_neg_thresholds:\n")
                for v in twist_neg_values:
                    f.write(f"{v}\n")

                f.write("\nsep_thresh_values:\n")
                for v in sep_thresh_values:
                    f.write(f"{v}\n")  
                
                f.write("\nsnr_thresh_values:\n") 
                for v in snr_thresh_values:
                    f.write(f"{v}\n")
                    
                f.write("\nRMS_values:\n")
                for v in RMS_values:
                    f.write(f"{v}\n")
                
                f.write("\ntone_flags:\n")
                for v in tone_flags:
                    f.write(f"{v}\n")

            print(f"Saved debug file: {filename}")

        except Exception as e:
            print("Error saving debug file:", e)

    def analyze_block(self, seg: np.ndarray, stabilizer, now_ms: float):

        if len(seg) != self.block:
            return None

        seg = seg.astype(float)
        seg -= seg.mean()

        seg_f = self.bp.process(seg)
        seg_f *= self.win

        E_low  = self.g_low.process(seg_f)
        E_high = self.g_high.process(seg_f)

        lf, l2 = self._top2(E_low,  FREQS_LOW)
        hf, h2 = self._top2(E_high, FREQS_HIGH)

        l_abs_db = db10(E_low[lf])
        l2_db    = db10(E_low[l2])
        h_abs_db = db10(E_high[hf])
        h2_db    = db10(E_high[h2])


        low_noise  = np.mean([E_low[f]  for f in FREQS_LOW  if f != lf])  + EPS
        high_noise = np.mean([E_high[f] for f in FREQS_HIGH if f != hf]) + EPS

        snr_low_raw  = db10(E_low[lf]  / low_noise)
        snr_high_raw = db10(E_high[hf] / high_noise)

        rms = np.sqrt(np.mean(seg_f**2)) + EPS
        
        # RAW unscaled metrics ->
        sep_low_raw  = l_abs_db - l2_db
        sep_high_raw = h_abs_db - h2_db
        twist_raw    = l_abs_db - h_abs_db

        # Apply RMS normalization ->
        sep_low  = sep_low_raw
        sep_high = sep_high_raw

        snr_low  = snr_low_raw
        snr_high = snr_high_raw

        twist = twist_raw

        # Adaptive thresholds and decisions ->
        sep_thresh = self.adaptive_sep_threshold(
            rms, self.sep_min, self.sep_max, rms_min=0.01, rms_max=0.1
        )
        sep_ok = (sep_low > sep_thresh) and (sep_high > sep_thresh)

        twist_pos_th, twist_neg_th = self.adaptive_twist_threshold(
            rms,
            twist_pos_max=self.twist_pos_max,
            twist_neg_min=self.twist_neg_min,
            twist_pos_default=self.twist_pos_db,
            twist_neg_default=self.twist_neg_db,
            rms_min=0.01,
            rms_max=0.1
        )
        twist_ok = (twist_neg_th <= twist <= twist_pos_th)

        snr_thresh = self.adaptive_snr_threshold(
            rms, self.snr_min, self.snr_max, rms_min=0.01, rms_max=0.1
        )
        
        snr_ok = (snr_low >= snr_thresh) and (snr_high >= snr_thresh)

        # Final decision ->
        good = sep_ok and twist_ok and snr_ok
        sym = LUT.get((lf, hf), "?") if good else "?"

        out = stabilizer.update(sym, now_ms)

        return sym, out, {
            "snr_low": snr_low,
            "snr_high": snr_high,
            "sep_low": sep_low,
            "sep_high": sep_high,
            "twist": twist,
            "twist_pos_threshold": twist_pos_th,
            "twist_neg_threshold": twist_neg_th,
            "sep_thresh": sep_thresh,
            "snr_thresh": snr_thresh,
            "rms": rms
        }

    def collect_plot_metrics(self, t_ms, block, sym, metrics,
                            amplitudes, block_symbols, SNR_values,
                            sep_db_values, twist_values, twist_neg_values, 
                            twist_pos_values, sep_thresh_values, snr_thresh_values,
                            RMS_values, tone_flags):

        if t_ms < 2000.0:
            return

        # amplitude
        amplitudes.extend(block.tolist())

        # symbolic detection
        block_symbols.append(sym if sym is not None else " ")

        # RMS value for this block
        rms = np.sqrt(np.mean(block.astype(float)**2))
        RMS_values.append(rms)

        # tone_present flag (1 = tone, 0 = no tone)
        tone_flags.append(1 if sym != "?" else 0)

        # SNR
        SNR_values.append(min(metrics['snr_low'], metrics['snr_high']))

        # SEP
        sep_db_values.append(min(metrics['sep_low'], metrics['sep_high']))

        # TWIST
        twist_values.append(metrics['twist'])
        twist_neg_values.append(metrics['twist_neg_threshold'])
        twist_pos_values.append(metrics['twist_pos_threshold'])

        # thresholds
        sep_thresh_values.append(metrics['sep_thresh'])
        snr_thresh_values.append(metrics['snr_thresh'])


    def stream_and_detect(self, stabilizer, sampler, plot=False):

        digits = []
        start_stage = 0
        collecting_payload = False

        t_ms = 0.0
        block_ms = 1000.0 * self.block / self.fs
        timer = 0.0

        amplitudes = []
        block_symbols = []
        SNR_values = []
        sep_db_values = []
        twist_values = []
        twist_neg_values = []
        twist_pos_values = []
        sep_thresh_values = []
        snr_thresh_values = []
        RMS_values = []
        tone_flags = []

        for block in sampler.stream_blocks(self.block):

            sym, out, metrics = self.analyze_block(block, stabilizer, t_ms)
            t_ms += block_ms

            # plotting
            self.collect_plot_metrics(
                t_ms, block, sym, metrics,
                amplitudes, block_symbols, SNR_values,sep_db_values,
                twist_values, twist_neg_values, twist_pos_values, 
                sep_thresh_values, snr_thresh_values, RMS_values, tone_flags)

            if collecting_payload:
                timer += block_ms

            if len(digits) < 8 and timer >= 6000.0 and len(digits) > 0:
                    print("Timeout while collecting digits → resetting")
                    digits.clear()
                    collecting_payload = False
                    start_stage = 0
                    timer = 0.0
                    self.save_plotting_txt(digits, amplitudes, block_symbols, SNR_values, sep_db_values
                                        ,twist_values, twist_neg_values, twist_pos_values, 
                                       sep_thresh_values, snr_thresh_values, RMS_values, tone_flags)
                    amplitudes.clear()
                    block_symbols.clear()
                    SNR_values.clear()
                    sep_db_values.clear()
                    twist_values.clear()
                    twist_neg_values.clear()
                    twist_pos_values.clear()
                    sep_thresh_values.clear()
                    snr_thresh_values.clear()
                    continue

            if not out:
                continue

            if not collecting_payload:

                # Waiting for "*"
                if start_stage == 0:
                    if out == "*":
                        digits.append(out)
                        start_stage = 1
                        print("Start '*' detected → waiting for '#'")
                    continue

                # Waiting for "#"
                elif start_stage == 1:
                    if out == "#":
                        digits.append(out)
                        collecting_payload = True
                        print("Second '#' detected → collecting digits...")
                    else:
                        print(f"Expected '#', got '{out}' → resetting")
                        digits.clear()
                        start_stage = 0
                    continue

            digits.append(out)
            print("Detected digits so far:", "".join(digits))

            if len(digits) == 8:
                # Save file after loop completes
                self.save_plotting_txt(digits, amplitudes, block_symbols, SNR_values, sep_db_values, 
                                       twist_values, twist_neg_values, twist_pos_values, 
                                       sep_thresh_values, snr_thresh_values, RMS_values, tone_flags)
                return "".join(digits)

    def stream_and_detect_duration(self, stabilizer, sampler, duration):
        digit = ""
        collecting_payload = False

        t_ms = 0.0
        block_ms = 1000.0 * self.block / self.fs
        max_time_ms = duration * 1000.0

        # plotting containers
        #amplitudes = []
        #block_symbols = []
        #SNR_values = []
        #sep_db_values = []
        #twist_values = []
        #twist_neg_values = []
        #twist_pos_values = []

        for block in sampler.stream_blocks(self.block):
            print(f"RMS amplitude: {np.sqrt(np.mean(block.astype(float)**2)):.5f}")

            if t_ms > max_time_ms:
                print("Duration exceeded, stopping detection.")
                break

            sym, out, metrics = self.analyze_block(block, stabilizer, t_ms)

            #self.collect_plot_metrics(
            #    t_ms, block, sym, metrics,
            #    amplitudes, block_symbols, SNR_values, sep_db_values, 
            #    twist_values, twist_neg_values, twist_pos_values
            #)

            t_ms += block_ms

            if not out:
                continue

            if not collecting_payload:

                if out == "B":
                    digit = out
                    print("ACK 'B' detected → stopping detection")
                    break

                elif out == "A":
                    digit = out
                    break

        # Save file after loop completes
        #self.save_plotting_txt(digit, amplitudes, block_symbols, SNR_values, sep_db_values, 
        #                       dom_db_values, twist_values, twist_neg_values, twist_pos_values)
        return digit

    def adaptive_twist_threshold(self, rms, 
                             twist_pos_max, twist_neg_min,
                             twist_pos_default, twist_neg_default, 
                             rms_min, rms_max): # Values twist_pos_max and twist_neg_min are overwritten in init

        if rms <= rms_min:
            # Very low RMS → no tone → be strict
            return twist_pos_default, twist_neg_default

        elif rms >= rms_max:
            # High RMS → tone present → be lenient
            return twist_pos_max, twist_neg_min

        else:
            # Interpolate from strict → lenient as RMS increases
            scale = np.sqrt((rms - rms_min) / (rms_max - rms_min))

            twist_pos = twist_pos_default + scale * (twist_pos_max - twist_pos_default)
            twist_neg = twist_neg_default + scale * (twist_neg_min - twist_neg_default)

            return twist_pos, twist_neg
    
    def adaptive_snr_threshold(self, rms, snr_min, snr_max, rms_min, rms_max):

        # Low RMS → need lenient threshold → use high (snr_max)
        if rms <= rms_min:
            return snr_max

        # High RMS → strict threshold → use low (snr_min)
        if rms >= rms_max:
            return snr_min

        # Interpolate in reverse
        scale = np.sqrt((rms - rms_min) / (rms_max - rms_min))
        return snr_max - scale * (snr_max - snr_min)



    def adaptive_sep_threshold(self, rms, sep_min, sep_max, rms_min, rms_max):

        if rms <= rms_min:
            return sep_max   # lenient threshold

        if rms >= rms_max:
            return sep_min   # strict threshold

        scale = np.sqrt((rms - rms_min) / (rms_max - rms_min))
        return sep_max - scale * (sep_max - sep_min)
    # Helper to find top 2 frequencies
    @staticmethod
    def _top2(energy_dict, freqs_tuple):
        top = sorted(freqs_tuple, key=lambda f: energy_dict[f], reverse=True)
        return top[0], top[1]
    