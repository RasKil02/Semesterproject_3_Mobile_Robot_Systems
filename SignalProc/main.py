from AudioSampling import AudioSampler
from BandpassFilter import BandPassFilter
from Goertzel import GoertzelAlgorithm
import numpy as np


FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = { # DTMF-tabel
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B', 
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

def hann(N): # Hann-vindue
    n = np.arange(N)
    return 0.5 * (1 - np.cos(2*np.pi*n/(N-1)))

def analyze(audio: np.ndarray, fs: int, block: int = 205):
    # forbered DSP
    bp = BandpassFilter(fs, 550, 1750, 4)
    g_low  = GoertzelAlgorithm(fs, FREQS_LOW)
    g_high = GoertzelAlgorithm(fs, FREQS_HIGH)
    win = hann(block)

    # simpel stabiliserings-logik: kræv 2 blokke i træk
    # last_sym = '?'
    # hold = 0

    nblocks = len(audio)//block
    for bi in range(nblocks):
        seg = audio[bi*block:(bi+1)*block].astype(np.float64)
        seg = seg - np.mean(seg)
        seg *= win
        seg_f = bp.process(seg)

        E_low  = g_low.process(seg_f)
        E_high = g_high.process(seg_f)
        low_f  = max(FREQS_LOW,  key=lambda f: E_low[f])
        high_f = max(FREQS_HIGH, key=lambda f: E_high[f])

        sym = LUT.get((low_f, high_f), '?')

        if sym == last_sym and sym != '?':
            hold += 1
            if hold == 2:  # ~50 ms ved 8 kHz og block=205
                t = (bi*block)/fs
                print(f"[{t:6.3f}s] DTMF: {sym}  (low={low_f}Hz, high={high_f}Hz)")
        else:
            last_sym = sym
            hold = 1 if sym != '?' else 0

def main():
    duration = 5
    fs = 8000           # skift gerne til 8000 for DTMF
    output = "output.wav"

    sampler = AudioSampler(duration, fs, output)
    freq_plot = BandPassFilter(fs)
    audio = sampler.record_audio()
    sampler.save_audio()
    sampler.plot_waveform()
    freq_plot.plot_freq_response(fs)

    analyze(audio, fs, block=205)

if __name__ == "__main__":
    main()