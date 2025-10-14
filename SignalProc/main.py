import argparse
import numpy as np
import matplotlib.pyplot as plt

from AudioSampling import AudioSampler
from BandpassFilter import BandPassFilter
from Goertzel import GoertzelAlgorithm

# Parametre for DTMF-detektion
MIN_DB   = -90.0   # absolut minimumsniveau pr. gruppe (dB) for at acceptere
SEP_DB   = 6.0     # min. afstand til næstbedste i samme gruppe (dB)
TWIST_DB = 8.0     # max forskel mellem row- og col-energi (dB)
HOLD_MIN = 2       # antal blokke i træk før vi udsender et symbol
GUARD_MS = 60      # mindste tid mellem samme symbol


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

def db(x: float) -> float:
    return 10*np.log10(max(x, 1e-12))

class DigitStabilizer:
    """Kræv hold_min blokke og guard mellem hits for rene impulser."""
    def __init__(self, hold_min=2, guard_ms=60):
        self.hold_min = hold_min
        self.guard_ms = guard_ms
        self.last = '?'
        self.hold = 0
        self.last_emit_ms = -1.0

    def update(self, sym, now_ms):
        if sym == '?':
            self.last = '?'
            self.hold = 0
            return None
        if sym == self.last:
            self.hold += 1
            if self.hold == self.hold_min:
                if self.last_emit_ms < 0 or (now_ms - self.last_emit_ms) > self.guard_ms:
                    self.last_emit_ms = now_ms
                    return sym
            return None
        else:
            self.last = sym
            self.hold = 1
            return None

def analyze_and_plot(audio: np.ndarray, fs: int, block: int = 205,
                     lowcut=650, highcut=1750, order=6, show_resp=False):
    from BandpassFilter import BandPassFilter
    from Goertzel import GoertzelAlgorithm

    bp = BandPassFilter(fs, lowcut, highcut, order)
    if show_resp: bp.plot_freq_response()
    g_low  = GoertzelAlgorithm(fs, block, FREQS_LOW)
    g_high = GoertzelAlgorithm(fs, block, FREQS_HIGH)
    win = hann(block)
    stab = DigitStabilizer()

    t_centers, row_db, col_db, emits = [], [], [], []

    nblocks = len(audio)//block
    for bi in range(nblocks):
        seg = audio[bi*block:(bi+1)*block].astype(float)
        seg -= seg.mean()
        seg *= win
        seg_f = bp.process(seg)

        E_low  = g_low.process(seg_f)
        E_high = g_high.process(seg_f)

        # SORTÉR for at få både #1 og #2 i hver gruppe
        low_sorted  = sorted(FREQS_LOW,  key=lambda f: E_low[f],  reverse=True)
        high_sorted = sorted(FREQS_HIGH, key=lambda f: E_high[f], reverse=True)
        lf, l2 = low_sorted[0],  low_sorted[1]
        hf, h2 = high_sorted[0], high_sorted[1]

        l_db, l2_db = db(E_low[lf]),  db(E_low[l2])
        h_db, h2_db = db(E_high[hf]), db(E_high[h2])

        t = (bi*block + block/2)/fs
        t_centers.append(t)
        row_db.append(l_db); col_db.append(h_db)

        abs_ok  = (l_db > MIN_DB) and (h_db > MIN_DB)
        sep_ok  = (l_db - l2_db > SEP_DB) and (h_db - h2_db > SEP_DB)
        twist_ok= abs(l_db - h_db) < TWIST_DB

        sym = LUT.get((lf, hf), '?') if (abs_ok and sep_ok and twist_ok) else '?'
        out = stab.update(sym, now_ms=t*1000.0)
        if out:
            emits.append((t, out))
            print(f"[{t:6.3f}s] DTMF: {out}  (low={lf}Hz, high={hf}Hz, "
                  f"row={l_db:.1f} dB, col={h_db:.1f} dB)")

    # -------- plots --------
    fig, axes = plt.subplots(2, 1, figsize=(10,7), sharex=True)
    axes[0].plot(t_centers, row_db, label="Row max (dB)")
    axes[0].plot(t_centers, col_db, label="Col max (dB)")
    axes[0].axhline(MIN_DB, color='k', ls='--', lw=1, label=f"ABS {MIN_DB} dB")
    axes[0].set_title("Max-energi pr. blok (efter band-pass)")
    axes[0].set_ylabel("dB"); axes[0].grid(True); axes[0].legend(loc="upper right")

    imp_t = np.array(t_centers); imp_y = np.zeros_like(imp_t)
    for t_emit, _sym in emits:
        idx = (np.abs(imp_t - t_emit)).argmin()
        imp_y[idx] = 1.0
    axes[1].stem(imp_t, imp_y)
    for t_emit, sym in emits:
        axes[1].annotate(sym, (t_emit, 1.02), ha="center", va="bottom", fontsize=12)
    axes[1].set_title("DTMF-detektion (impulser)")
    axes[1].set_xlabel("Tid [s]"); axes[1].set_ylim(0,1.2); axes[1].grid(True)

    plt.tight_layout(); plt.show()

def main():
    ap = argparse.ArgumentParser(description="DTMF analyse (optag og plot impulser).")
    ap.add_argument("--duration", type=float, default=5.0, help="Optagetid i sek.")
    ap.add_argument("--fs", type=int, default=48000, help="Samplerate (Hz)")
    ap.add_argument("--block", type=int, default=205, help="Bloklængde til Goertzel")
    ap.add_argument("--out", type=str, default="output.wav", help="Gem WAV som")
    ap.add_argument("--show-filter", action="store_true", help="Vis filterets frekvensrespons")
    args = ap.parse_args()

    # 1) Optag og gem + vis waveform
    sampler = AudioSampler(args.duration, args.fs, args.out)
    audio = sampler.record_audio()
    sampler.save_audio()
    sampler.plot_waveform()

    # 2) Analyse + impulser
    analyze_and_plot(audio, fs=args.fs, block=args.block, show_resp=args.show_filter)

if __name__ == "__main__":
    main()
