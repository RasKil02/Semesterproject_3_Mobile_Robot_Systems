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
                 hop_ms: float   = 10.0,
                 lowcut: float   = 620.0,
                 highcut: float  = 1700.0,
                 bp_order: int   = 4,
                 # Tærskler
                 min_db: float = -20.0,     # minimum absolut db
                 sep_db: float = 7.0,       # separations-tærskel
                 dom_db: float = 12.0,       # dominans-tærskel
                 snr_db: float = 12.0,       # SNR-tærskel
                 twist_pos_db: float = +5.0,   # positiv twist grænse (row > col)
                 twist_neg_db: float = -5.0,
                 twist_pos_max: float = +30,
                 twist_neg_min: float = -30):  # negativ twist grænse (col > row)

        self.fs = int(fs)
        self.block = max(1, int(self.fs * (block_ms/1000.0))) # 240 samples ved 30 ms @ 8kHz
        self.hop   = max(1, int(self.fs * (hop_ms/1000.0))) # 60 samples ved 7.5 ms @ 8kHz

        # filter + vindue
        self.bp = BandPassFilter(self.fs, lowcut, highcut, bp_order) # Uses butterworth bandpass filter
        self.win = hann(self.block) # Sets up hanning window to be used on each block

        # goertzel pr. gruppe
        self.g_low  = GoertzelAlgorithm(self.fs, self.block, FREQS_LOW) # Will measure the power at the low DTMF frequencies inside each block
        self.g_high = GoertzelAlgorithm(self.fs, self.block, FREQS_HIGH) # Will measure the power at the high DTMF frequencies inside each block

        # Thresholds
        self.min_db  = float(min_db)
        self.sep_db  = float(sep_db)
        self.dom_db  = float(dom_db)
        self.snr_db  = float(snr_db)
        self.twist_pos_db = float(twist_pos_db)
        self.twist_neg_db = float(twist_neg_db)
        self.twist_pos_max = float(twist_pos_max)
        self.twist_neg_min = float(twist_neg_min)

    def save_plotting_txt(self, digits, amplitudes, block_symbols, SNR_values,
                        sep_db_values, dom_db_values, twist_values):

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
                f.write(f"min_db threshold: {self.min_db}\n")
                f.write(f"sep_db threshold: {self.sep_db}\n")
                f.write(f"dom_db threshold: {self.dom_db}\n")
                f.write(f"snr_db threshold: {self.snr_db}\n")
                f.write(f"twist_pos_db: {self.twist_pos_db}\n")
                f.write(f"twist_neg_db: {self.twist_neg_db}\n")

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

                f.write("\ndom_db values:\n")
                for v in dom_db_values:
                    f.write(f"{v}\n")

                f.write("\ntwist values:\n")
                for v in twist_values:
                    f.write(f"{v}\n")

            print(f"Saved debug file: {filename}")

        except Exception as e:
            print("Error saving debug file:", e)


    # Analyze audio data for DTMF digits
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

        blk_db   = db10(np.mean(seg_f**2))
        l_abs_db = db10(E_low[lf])
        l2_db    = db10(E_low[l2])
        h_abs_db = db10(E_high[hf])
        h2_db    = db10(E_high[h2])

        low_noise  = np.mean([E_low[f]  for f in FREQS_LOW  if f != lf])  + EPS
        high_noise = np.mean([E_high[f] for f in FREQS_HIGH if f != hf]) + EPS

        snr_low_db  = db10(E_low[lf]  / low_noise)
        snr_high_db = db10(E_high[hf] / high_noise)

        sep_ok = (l_abs_db - l2_db > self.sep_db) and (h_abs_db - h2_db > self.sep_db)
        abs_ok = (l_abs_db - blk_db > self.min_db) and (h_abs_db - blk_db > self.min_db)

        twist = (l_abs_db - h_abs_db)

        rms = np.sqrt(np.mean(seg_f**2)) + EPS

        twist_pos_threshold, twist_neg_threshold = self.adaptive_twist_threshold(rms, 
                                                                                rms_min=0.01, 
                                                                                rms_max=0.1,
                                                                                twist_pos_max=self.twist_pos_max,
                                                                                twist_neg_min=self.twist_neg_min,
                                                                                twist_pos_default=self.twist_pos_db,
                                                                                twist_neg_default=self.twist_neg_db)

        twist_ok = (twist_neg_threshold <= twist <= twist_pos_threshold)

        l_dom_db = db10(E_low[lf]  / low_noise)
        h_dom_db = db10(E_high[hf] / high_noise)
        dom_ok   = (l_dom_db >= self.dom_db) and (h_dom_db >= self.dom_db)

        snr_ok   = (snr_low_db >= self.snr_db) and (snr_high_db >= self.snr_db)

        good = abs_ok and sep_ok and twist_ok and dom_ok and snr_ok

        sym = LUT.get((lf, hf), "?") if good else "?"

        # IMPORTANT: use stabilizer exactly like your old analyze()
        out = stabilizer.update(sym, now_ms=now_ms)

        return sym, out, {
            "snr_low": snr_low_db,
            "snr_high": snr_high_db,
            "min_db_low": l_abs_db - blk_db,
            "min_db_high": h_abs_db - blk_db,
            "sep_low": l_abs_db - l2_db,
            "sep_high": h_abs_db - h2_db,
            "dom_low": l_dom_db,
            "dom_high": h_dom_db,
            "twist": twist,
            "twist_pos_threshold": twist_pos_threshold,
            "twist_neg_threshold": twist_neg_threshold}
    
    def collect_plot_metrics(self, t_ms, block, sym, metrics,
                         amplitudes, block_symbols, SNR_values,
                         min_db_values, sep_db_values,
                         dom_db_values, twist_values, twist_neg_values, twist_pos_values):

        if t_ms < 2000.0:
            return

        amplitudes.extend(block.tolist())
        block_symbols.append(sym if sym is not None else " ")

        # SNR
        SNR_values.append(min(metrics['snr_low'], metrics['snr_high']))
        # min_db
        min_db_values.append(min(metrics['min_db_low'], metrics['min_db_high']))
        # separation metric
        sep_db_values.append(min(metrics['sep_low'], metrics['sep_high']))
        # dominance
        dom_db_values.append(min(metrics['dom_low'], metrics['dom_high']))
        # twist
        twist_values.append(metrics['twist'])
        twist_neg_values.append(metrics['twist_neg_threshold'])
        twist_pos_values.append(metrics['twist_pos_threshold'])

    
    def stream_and_detect(self, stabilizer, sampler, plot=False):

        digits = []
        start_stage = 0
        collecting_payload = False

        t_ms = 0.0
        block_ms = 1000.0 * self.block / self.fs

        amplitudes = []
        block_symbols = []
        SNR_values = []
        min_db_values = []
        sep_db_values = []
        dom_db_values = []
        twist_values = []
        twist_neg_values = []
        twist_pos_values = []

        for block in sampler.stream_blocks(self.block):
            # rms = np.sqrt(np.mean(block**2))
            # print(f"RMS: {rms:.5f}")

            sym, out, metrics = self.analyze_block(block, stabilizer, t_ms)
            t_ms += block_ms

            # NEW clean call
            self.collect_plot_metrics(
                t_ms, block, sym, metrics,
                amplitudes, block_symbols, SNR_values,
                min_db_values, sep_db_values, dom_db_values, twist_values, twist_neg_values, twist_pos_values
            )

            if not out:
                continue

            if not collecting_payload:

                if start_stage == 0:
                    if out == "*":
                        digits.append(out)
                        start_stage = 1
                        print("Start '*' detected → waiting for '#'")
                    continue

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

            # collect payload digits
            digits.append(out)
            print("Detected digits so far:", "".join(digits))

            if len(digits) == 8:

                self.save_plotting_txt(
                    digits, amplitudes, block_symbols, SNR_values,
                    sep_db_values, dom_db_values, twist_values
                )

                return "".join(digits)

            
    def stream_and_detect_duration(self, stabilizer, sampler, duration):
        digit = ""
        collecting_payload = False

        t_ms = 0.0
        block_ms = 1000.0 * self.block / self.fs
        max_time_ms = duration * 1000.0

        # plotting containers
        amplitudes = []
        block_symbols = []
        SNR_values = []
        min_db_values = []
        sep_db_values = []
        dom_db_values = []
        twist_values = []
        twist_neg_values = []
        twist_pos_values = []

        for block in sampler.stream_blocks(self.block):

            if t_ms > max_time_ms:
                print("Duration exceeded, stopping detection.")
                break

            sym, out, metrics = self.analyze_block(block, stabilizer, t_ms)

            self.collect_plot_metrics(
                t_ms, block, sym, metrics,
                amplitudes, block_symbols, SNR_values,
                min_db_values, sep_db_values, dom_db_values, twist_values, twist_neg_values, twist_pos_values
            )

            t_ms += block_ms

            if not out:
                continue

            if not collecting_payload:

                if out == "B":
                    digit = out
                    print("ACK 'B' detected → stopping detection")
                    collecting_payload = True

                elif out == "A":
                    digit = out
                    collecting_payload = True

        # Save file after loop completes
        self.save_plotting_txt(digit, amplitudes, block_symbols, SNR_values, sep_db_values, dom_db_values, twist_values)
        return digit

    def adaptive_twist_threshold(self, rms, rms_min=0.01, rms_max=0.1,
                             twist_pos_max=30.0, twist_neg_min=-30.0,
                             twist_pos_default=5.0, twist_neg_default=-5.0):

        if rms <= rms_min:
            # Lowest RMS, widest threshold
            return twist_pos_max, twist_neg_min
        elif rms >= rms_max:
            # Highest RMS, narrowest threshold (default)
            return twist_pos_default, twist_neg_default
        else:
            # Linearly interpolate thresholds between max and default
            scale = (rms - rms_min) / (rms_max - rms_min)
            twist_pos = twist_pos_max - scale * (twist_pos_max - twist_pos_default)
            twist_neg = twist_neg_min + scale * (twist_neg_default - twist_neg_min)
            return twist_pos, twist_neg



    # Helper to find top 2 frequencies
    @staticmethod
    def _top2(energy_dict, freqs_tuple):
        top = sorted(freqs_tuple, key=lambda f: energy_dict[f], reverse=True)
        return top[0], top[1]
    