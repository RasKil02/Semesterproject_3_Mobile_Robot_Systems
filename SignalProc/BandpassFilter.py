# SignalProc/BandpassFilter.py
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, sosfreqz
import matplotlib.pyplot as plt

class BandPassFilter:
    def __init__(self, fs: int, low: float = 550.0, high: float = 1750.0, order: int = 4):
        self.fs = fs
        nyq = 0.5 * fs
        if not (0 < low < high < nyq):
            raise ValueError("Frekvensgrænser skal være 0 < low < high < fs/2")
        wn = [low/nyq, high/nyq]
        self.sos = butter(order, wn, btype="band", output="sos")  # shape: (n_sections, 6)
        
        # init-tilstand til HELE filteret (alle sektioner)
        self._zi = None # udskydes til første kørsel

    def process(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float32).squeeze()
        if x.ndim != 1:
            raise ValueError("BandPassFilter.process forventer 1D signal.")
        if self._zi is None:
            # skaler zi med første sample for at undgå opstarts-transient
            zi0 = sosfilt_zi(self.sos) * float(x[0])
            y, self._zi = sosfilt(self.sos, x, zi=zi0)
        else:
            y, self._zi = sosfilt(self.sos, x, zi=self._zi)
        return y

    def plot_freq_response(self):
        w, h = sosfreqz(self.sos, worN=2000, fs=self.fs)
        plt.figure()
        # vis i dB på en log-frekvensakse (pænere at læse)
        mag_db = 20*np.log10(np.maximum(np.abs(h), 1e-12))
        plt.semilogx(w, mag_db)
        plt.title("Bandpass Filter Frequency Response")
        plt.xlabel("Frequency [Hz]")
        plt.ylabel("Gain [dB]")
        plt.grid(True, which="both", ls=":")
        plt.tight_layout()
        plt.show()
