import numpy as np
from SignalProc.BandpassFilter import BandPassFilter
from SignalProc.Goertzel import GoertzelAlgorithm
from SignalProc.AudioSampling import AudioSampler

FREQS_LOW  = (697, 770, 852, 941)
FREQS_HIGH = (1209, 1336, 1477, 1633)
LUT = {
    (697,1209):'1',(697,1336):'2',(697,1477):'3',(697,1633):'A',
    (770,1209):'4',(770,1336):'5',(770,1477):'6',(770,1633):'B',
    (852,1209):'7',(852,1336):'8',(852,1477):'9',(852,1633):'C',
    (941,1209):'*',(941,1336):'0',(941,1477):'#',(941,1633):'D'
}

def hann(N):
    n = np.arange(N)
    return 0.5 * (1 - np.cos(2 * np.pi * n / (N - 1)))

class DTMFDetector:
    def __init__(self, fs=8000, block_ms=30.0, hop_ms=7.5, lowcut=620, highcut=1700, bp_order=4):
        self.fs = int(fs)
        self.block = max(1, int(self.fs * (block_ms / 1000.0)))
        self.hop = max(1, int(self.fs * (hop_ms / 1000.0)))
        self.bp = BandPassFilter(self.fs, lowcut, highcut, bp_order)
        self.win = hann(self.block)
        self.g_low = GoertzelAlgorithm(self.fs, self.block, FREQS_LOW)
        self.g_high = GoertzelAlgorithm(self.fs, self.block, FREQS_HIGH)

    def record_and_detect(self, duration_s, out_wav, stabilizer=None):
        sampler = AudioSampler(duration_s, self.fs, out_wav)
        audio = sampler.record_audio()
        sampler.save_audio()
        return self.analyze(audio, stabilizer)

    def analyze(self, audio, stabilizer=None):
        digits = []
        nblocks = 1 + max(0, (len(audio) - self.block) // self.hop)

        for bi in range(nblocks):
            start = bi * self.hop
            seg = audio[start:start + self.block]
            if len(seg) < self.block:
                break

            seg -= np.mean(seg)
            seg_f = self.bp.process(seg) * self.win

            E_low = self.g_low.process(seg_f)
            E_high = self.g_high.process(seg_f)

            lf = max(E_low, key=E_low.get)
            hf = max(E_high, key=E_high.get)
            sym = LUT.get((lf, hf), "?")

            t_ms = (start + self.block / 2) / self.fs * 1000.0
            if stabilizer:
                out = stabilizer.update(sym, now_ms=t_ms)
                if out and (not digits or digits[-1] != out):
                    digits.append(out)
            else:
                if sym != "?" and (not digits or digits[-1] != sym):
                    digits.append(sym)

        return "".join(digits)
