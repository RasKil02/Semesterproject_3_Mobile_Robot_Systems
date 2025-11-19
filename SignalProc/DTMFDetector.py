# dtmf_detector.py
import numpy as np
import argparse
from SignalProc.BandpassFilter import BandPassFilter
from SignalProc.Goertzel import GoertzelAlgorithm
from SignalProc.AudioSampling import AudioSampler
from SignalProc.Plotting import Plotting

# --- DTMF opsætning ---
FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = {
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B',
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

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
    def __init__(self, hold_ms=30, miss_ms=20, gap_ms=55):
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
                 fs: int,
                 block_ms: float = 30.0,
                 hop_ms: float   = 7.5,
                 lowcut: float   = 620.0,
                 highcut: float  = 1700.0,
                 bp_order: int   = 4,
                 # Tærskler
                 min_db: float = -20.0,     # minimum absolut db
                 sep_db: float = 5.0,       # separations-tærskel
                 dom_db: float = 4.0,       # dominans-tærskel
                 snr_db: float = 8.0,       # SNR-tærskel
                 twist_pos_db: float = +30.0,   # positiv twist grænse (row > col)
                 twist_neg_db: float = -30.0):  # negativ twist grænse (col > row)
        
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
        twist_ok = (self.twist_neg_db <= twist <= self.twist_pos_db)

        l_dom_db = db10(E_low[lf]  / low_noise)
        h_dom_db = db10(E_high[hf] / high_noise)
        dom_ok   = (l_dom_db >= self.dom_db) and (h_dom_db >= self.dom_db)

        snr_ok   = (snr_low_db >= self.snr_db) and (snr_high_db >= self.snr_db)

        good = abs_ok and sep_ok and twist_ok and dom_ok and snr_ok

        sym = LUT.get((lf, hf), "?") if good else "?"

        # IMPORTANT: use stabilizer exactly like your old analyze()
        out = stabilizer.update(sym, now_ms=now_ms)

        return out

    def stream_and_detect(self, stabilizer, sampler):
        
        digits = []
        start_stage = 0       # 0 = waiting for '*', 1 = waiting for '#'
        collecting_payload = False

        t_ms = 0.0
        block_ms = 1000.0 * self.block / self.fs

        for block in sampler.stream_blocks(self.block):
            out = self.analyze_block(block, stabilizer, t_ms)
            t_ms += block_ms

            if not out:
                continue

            if not collecting_payload:

                if start_stage == 0:
                    # Expecting first start bit '*'
                    if out == "*":
                        digits.append(out)
                        start_stage = 1
                        print("Start symbol '*' detected, waiting for '#'...")
                    # ignore everything else
                    continue

                elif start_stage == 1:
                    # Expecting second start bit '#'
                    if out == "#":
                        digits.append(out)
                        collecting_payload = True
                        print("Second symbol '#' detected, collecting payload digits...")
                    else:
                        # WRONG second symbol → reset to stage 0
                        print(f"Expected '#', but got '{out}'. Resetting to wait for '*'.")
                        digits.clear()
                        start_stage = 0
                    continue
                
            digits.append(out)
            
            print(f"Detected digits so far: {''.join(digits)}")

            if len(digits) == 7:
                return "".join(digits)

    # Helper to find top 2 frequencies
    @staticmethod
    def _top2(energy_dict, freqs_tuple):
        top = sorted(freqs_tuple, key=lambda f: energy_dict[f], reverse=True)
        return top[0], top[1]