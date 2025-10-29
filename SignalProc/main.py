import argparse
import numpy as np
import matplotlib.pyplot as plt

from AudioSampling import AudioSampler
from BandpassFilter import BandPassFilter
from Goertzel import GoertzelAlgorithm

# Parametre for DTMF-detektion
MIN_DB   = -20.0   # absolut minimumsniveau pr. gruppe (dB) for at acceptere
SEP_DB   = 5       # min. afstand til næstbedste i samme gruppe (dB)
TWIST_DB = 8       # max forskel mellem row- og col-energi (dB)
DOM_DB   = 4       # min. dominerende forskel (dB) for at acceptere
SNR_DB   = 8      # min. SNR (Signal-To-Noise Ratio) i dB for at acceptere


# DTMF-frekvenser og tabel
FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = {
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B',
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

def hann(N: int):
    n = np.arange(N)
    return 0.5 * (1 - np.cos(2*np.pi*n/(N-1)))

def db10(x: float) -> float:
    return 10*np.log10(max(x, 1e-12))

class DigitStabilizer:
    """Kræv hold_min blokke og guard mellem hits for rene impulser."""
    # hold_ms = tonens minimum varighed for at kunne detekteres
    # miss_ms = hvor længe et udfald må vare uden at miste låsen
    # gap_ms  = minimum stilhed mellem to symboler
    def __init__(self, hold_ms, miss_ms, gap_ms):
        self.hold_ms = float(hold_ms)
        self.miss_ms = float(miss_ms)
        self.gap_ms  = float(gap_ms)

        self.state = "IDLE"      # IDLE | CANDIDATE | LOCKED | GAP
        self.curr = None         # kandidat/låst symbol
        self.last_locked = None    # sidste låste symbol
        self.t_start = None      # starttid for kandidat
        self.t_last_seen = None  # sidste gang vi så curr (til miss_ms)
        self.t_emit = None       # tidspunkt for seneste emit
        self.gap_start = None    # starttid for GAP

    def update(self, sym: str, now_ms: float):
        # Normalisér til gyldige symboler
        if sym not in "1234567890ABCD*#":
            sym = "?"

        if self.state == "IDLE":
            if sym == "?":
                return None
            # start kandidat
            self.state = "CANDIDATE"
            self.curr = sym
            self.t_start = now_ms
            self.t_last_seen = now_ms
            return None

    # --- CANDIDATE ---------------------------------------------------------
        if self.state == "CANDIDATE":
            if sym == self.curr:
                self.t_last_seen = now_ms
                # krav: stabil tid >= hold_ms
                if (now_ms - self.t_start) >= self.hold_ms:
                    # lås og emit én gang
                    self.state = "LOCKED"
                    self.t_emit = now_ms
                    self.last_locked = self.curr
                    return self.curr
                return None
            elif sym == "?":
                # mistet før lås → reset
                self.state = "IDLE"
                self.curr = None
                self.t_start = None
                self.t_last_seen = None
                self.gap_start = None
                return None
            else:
                # ny kandidat
                self.curr = sym
                self.t_start = now_ms
                self.t_last_seen = now_ms
                return None

    # --- LOCKED ------------------------------------------------------------
        if self.state == "LOCKED":
            if sym == self.curr:
                # stadig låst; ingen ny emit
                self.t_last_seen = now_ms
                return None
            elif sym == "?":
                # tillad korte udfald uden at slippe låsen
                if (now_ms - self.t_last_seen) > self.miss_ms:
                    # slip lås og kræv stilhed (GAP) før næste symbol
                    self.state = "GAP"
                    self.gap_start = now_ms
                return None
            else:
                # andet symbol dukker op uden stilhed -> ny kandidat    
                self.state = "CANDIDATE"
                self.curr = sym
                self.t_start = now_ms
                self.t_last_seen = now_ms
                return None

    # --- GAP ---------------------------------------------------------------
        if self.state == "GAP":
            # vi kræver stilhed i mindst gap_ms før nyt symbol må overvejes
            if sym == "?":
                if self.gap_start is None:
                    self.gap_start = now_ms
                if (now_ms - (self.gap_start or now_ms)) >= self.gap_ms:
                    # klar til ny kandidat
                    self.state = "IDLE"
                    self.curr = None
                    self.t_start = None
                    self.t_last_seen = None
                return None
            else:
                # Hvis samme som sidst låste: kræv stilhed (hindrer dobbeltskud)
                if self.last_locked is not None and sym == self.last_locked:
                    self.t_last_seen = now_ms
                    return None 
                else:
                    # Start ny kandidat med det nye symbol
                    self.state = "CANDIDATE"
                    self.curr = sym
                    self.t_start = now_ms
                    self.t_last_seen = now_ms
                return None
            return None

def analyze(audio: np.ndarray, fs: int, block: int, hop: int = None,
                     lowcut=620, highcut=1700, order=6, show_resp=False):
    if hop is None:
        hop = block  # ingen overlap som standard

    bp = BandPassFilter(fs, lowcut, highcut, order)
    if show_resp: bp.plot_freq_response()
    g_low  = GoertzelAlgorithm(fs, block, FREQS_LOW)
    g_high = GoertzelAlgorithm(fs, block, FREQS_HIGH)
    win = hann(block)
    stab = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    t_centers, row_db, col_db, emits = [], [], [], []

    detected_digits = [] # liste af (digit)

    nblocks = 1 + max(0, (len(audio) - block) // hop)
    for bi in range(nblocks):
        start = bi*hop
        seg = audio[start:start+block].astype(float)
        if len(seg) < block:
            break  # ufuldstændig blok i slutningen

        seg -= seg.mean()        # fjern DC
        seg_f = bp.process(seg)  # bandpass-filter
        seg_f *= win             # vindue

        E_low  = g_low.process(seg_f)    # energi pr. frekvens
        E_high = g_high.process(seg_f)   # energi pr. frekvens

        # SORTÉR for at få både #1 og #2 i hver gruppe
        low_sorted  = sorted(FREQS_LOW,  key=lambda f: E_low[f],  reverse=True)
        high_sorted = sorted(FREQS_HIGH, key=lambda f: E_high[f], reverse=True)
        lf, l2 = low_sorted[0],  low_sorted[1]
        hf, h2 = high_sorted[0], high_sorted[1]

        blk_db = db10(np.mean(seg_f**2))

        l_abs_db = db10(E_low[lf])
        h_abs_db = db10(E_high[hf])
        l2_db    = db10(E_low[l2])
        h2_db    = db10(E_high[h2])

        # Relative dB ift. blok-RMS (matcher din ABS-test)
        l_rel_db = l_abs_db - blk_db
        h_rel_db = h_abs_db - blk_db

        t = (start + block/2) / fs  # center-tid i sek
        t_centers.append(t)
        row_db.append(l_rel_db); col_db.append(h_rel_db)

        low_noise  = np.mean([E_low[f]  for f in FREQS_LOW  if f != lf])  + 1e-12
        high_noise = np.mean([E_high[f] for f in FREQS_HIGH if f != hf]) + 1e-12
        snr_low_db  = db10((E_low[lf]) / low_noise)
        snr_high_db = db10((E_high[hf]) / high_noise)
        snr_ok = (snr_low_db >= SNR_DB) and (snr_high_db >= SNR_DB)

        sep_ok   = (l_abs_db - l2_db > SEP_DB) and (h_abs_db - h2_db > SEP_DB)
        abs_ok   = (l_abs_db - blk_db > MIN_DB) and (h_abs_db - blk_db > MIN_DB)
        twist = (l_abs_db - h_abs_db)
        twist_ok = (-TWIST_DB <= twist <= TWIST_DB/2) # tillad lidt mere lavfrekvens-domineret

        # Dominans
        l_dom_db = db10(E_low[lf] / low_noise)
        h_dom_db = db10(E_high[hf] / high_noise)
        dom_ok = (l_dom_db >= DOM_DB) and (h_dom_db >= DOM_DB)

        good = abs_ok and sep_ok and twist_ok and dom_ok and snr_ok
        sym = LUT.get((lf, hf), "?") if good else "?"

        out = stab.update(sym, now_ms=t*1000.0)
        if out:
            if not detected_digits or detected_digits[-1] != out:
                detected_digits.append(out)

    if detected_digits:
        print("\n--- Detected digits ---")
        print("".join(detected_digits))
    else:
        print("--- No digits detected ---")

def main():
    ap = argparse.ArgumentParser(description="DTMF analyse (optag og plot impulser).")
    ap.add_argument("--duration", type=float, default=4.0, help="Optagetid i sek.")
    ap.add_argument("--fs", type=int, default=8000, help="Samplerate (Hz)")
    ap.add_argument("--block", type=int, default=0, help="Bloklængde i samples (0=4ms)")
    ap.add_argument("--out", type=str, default="output.wav", help="Gem WAV som")
    ap.add_argument("--show-filter", action="store_true", help="Vis filterets frekvensrespons")
    args = ap.parse_args()

    # 1) Optag og gem + vis waveform
    sampler = AudioSampler(args.duration, args.fs, args.out)
    audio = sampler.record_audio()
    sampler.save_audio()

    # 2) Analyse + impulser
    block = args.block if args.block > 0 else int(0.030 * sampler.fs) # 30 ms blok
    hop = max(1, int(0.075 * sampler.fs))               # 7.5 ms hop (overlap)
    analyze(audio, fs=sampler.fs, block=block, hop=hop, show_resp=args.show_filter)

if __name__ == "__main__":
    main()
