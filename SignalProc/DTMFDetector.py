# dtmf_detector.py
import numpy as np
import argparse
from BandpassFilter import BandPassFilter
from Goertzel import GoertzelAlgorithm
from AudioSampling import AudioSampler

# --- DTMF opsætning ---
FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = {
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B',
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

# Sets up hann window
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
                 twist_pos_db: float = +4.0,   # positiv twist grænse (row > col)
                 twist_neg_db: float = -8.0):  # negativ twist grænse (col > row)
        
        self.fs = int(fs)
        self.block = max(1, int(self.fs * (block_ms/1000.0))) # 240 samples ved 30 ms @ 8kHz
        self.hop   = max(1, int(self.fs * (hop_ms/1000.0))) # 60 samples ved 7.5 ms @ 8kHz

        # filter + vindue
        self.bp = BandPassFilter(self.fs, lowcut, highcut, bp_order) # Uses butterworth bandpass filter
        self.win = hann(self.block) # Sets up hann window to be used on each block

        # goertzel pr. gruppe
        self.g_low  = GoertzelAlgorithm(self.fs, self.block, FREQS_LOW) # Will measure the power at the low DTMF frequencies inside each block
        self.g_high = GoertzelAlgorithm(self.fs, self.block, FREQS_HIGH) # Will measure the power at the high DTMF frequencies inside each block

        # tærskler
        self.min_db  = float(min_db)
        self.sep_db  = float(sep_db)
        self.dom_db  = float(dom_db)
        self.snr_db  = float(snr_db)
        self.twist_pos_db = float(twist_pos_db)
        self.twist_neg_db = float(twist_neg_db)

    # Recordér audio og detektér DTMF cifre
    def record_and_detect(self, duration_s: float, out_wav: str, stabilizer) -> str:
        sampler = AudioSampler(duration_s, self.fs, out_wav) # Sets up audio sampler (instance of AudioSampler class)
        audio = sampler.record_audio() # Uses the record_audio method to record audio for a set duration
        sampler.save_audio()
        return self.analyze(audio, stabilizer=stabilizer) # Analyzes the recorded audio and returns detected digits

    # Analyser audio og returnér detekterede cifre som streng, takes numpy array as input and returns string
    def analyze(self, audio: np.ndarray, stabilizer) -> str:
        # Digits holds detected digits, later to be joined and returned as string
        digits = []
        
        nblocks = 1 + max(0, (len(audio) - self.block) // self.hop) # Number of analysis blocks, based on audio length, block size and hop size, so some of these will overlap
        
        # Loop over all blocks 
        for bi in range(nblocks):
            start = bi * self.hop
            seg = audio[start:start+self.block].astype(float) # Extract current block from audio signal, seg now holds the current block samples
            
            if len(seg) < self.block:
                break

            seg -= seg.mean() # Remove DC offset
            seg_f = self.bp.process(seg) # self.bp is an instance of BandPassFilter, this applies the bandpass filter to the current block, using its process method
            seg_f *= self.win # Apply windowing to the filtered block

            E_low  = self.g_low.process(seg_f) # Measure power at low DTMF frequencies using Goertzel algorithm
            E_high = self.g_high.process(seg_f) # Same for high DTMF frequencies

            # So now we have the energies at each DTMF frequency for the current block
            # Which could look something like:
            # E_low  = {697: 0.01, 770: 0.5, 852: 0.02, 941: 0.03}
            # E_high = {1209: 0.02, 1336: 0.6, 1477: 0.01, 1633: 0.03}
            # Which would indicate a DTMF '5' (770 Hz and 1336 Hz)
            
            # We have to find the two highest energies in each group
            # We use the helper method _top2 defined below, which returns the frequencies with the highest and second highest energies
            # So lf is the low frequency with highest energy, l2 is the second highest
            # In the example above, lf would be 770 and l2 would be 941, and similarly for hf and h2 we would get 1336 and 1633
            # Which would indicate a DTMF '5' (770 Hz and 1336 Hz)
            lf, l2 = self._top2(E_low,  FREQS_LOW)
            hf, h2 = self._top2(E_high, FREQS_HIGH)

            blk_db   = db10(np.mean(seg_f**2)) # Average block energy in dB
            
            l_abs_db = db10(E_low[lf]); # decibal level of the dominant low frequency
            l2_db = db10(E_low[l2]) # decibal level of the second dominant low frequency
            
            h_abs_db = db10(E_high[hf]); # for dominant high frequency
            h2_db = db10(E_high[h2])
            
            # Now we have the decibal levels of the dominant and second dominant frequencies in both low and high groups
            
            # Calculate background noise levels by averaging energies of every frequency except the dominant one, for both low and high groups
            low_noise  = np.mean([E_low[f]  for f in FREQS_LOW  if f != lf])  + EPS
            high_noise = np.mean([E_high[f] for f in FREQS_HIGH if f != hf]) + EPS
            
            # Calculate SNR for both low and high frequencies, SNR (signal to noise ratio), indicates how much stronger the signal is compared to the background noise
            snr_low_db  = db10(E_low[lf]  / low_noise)
            snr_high_db = db10(E_high[hf] / high_noise)

            sep_ok = (l_abs_db - l2_db > self.sep_db) and (h_abs_db - h2_db > self.sep_db)
            abs_ok = (l_abs_db - blk_db > self.min_db) and (h_abs_db - blk_db > self.min_db)
            twist  = (l_abs_db - h_abs_db)
            twist_ok = (self.twist_neg_db <= twist <= self.twist_pos_db)

            l_dom_db = db10(E_low[lf]  / low_noise)
            h_dom_db = db10(E_high[hf] / high_noise)
            dom_ok   = (l_dom_db >= self.dom_db) and (h_dom_db >= self.dom_db)

            snr_ok   = (snr_low_db >= self.snr_db) and (snr_high_db >= self.snr_db)

            good = abs_ok and sep_ok and twist_ok and dom_ok and snr_ok
            sym = LUT.get((lf, hf), "?") if good else "?"

            t_ms = (start + self.block/2) / self.fs * 1000.0
            out = stabilizer.update(sym, now_ms=t_ms)
            if out and (not digits or digits[-1] != out):
                digits.append(out)

        return "".join(digits)

    # Helper to find top 2 frequencies
    @staticmethod
    def _top2(energy_dict, freqs_tuple):
        top = sorted(freqs_tuple, key=lambda f: energy_dict[f], reverse=True)
        return top[0], top[1]