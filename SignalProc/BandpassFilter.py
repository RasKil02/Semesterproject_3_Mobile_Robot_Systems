# SignalProc/BandpassFilter.py
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi, sosfreqz
import matplotlib.pyplot as plt

class BandPassFilter:
    def __init__(self, fs: int, low: float = 600.0, high: float = 1700.0, order: int = 4):
        self.fs = fs
        nyq = 0.5 * fs   # Nyquist-frekvens
        
        # Validering af frekvensgrænser
        if not (0 < low < high < nyq):
            raise ValueError("Frekvensgrænser skal være 0 < low < high < fs/2")
        
        wn = [low/nyq, high/nyq] # Normaliserede grænser
        
        # Design af bandpass-filteret i sektioner
        self.sos = butter(order, wn, btype="band", output="sos")  # shape: (n_sections, 6)
        
        # init-tilstand til HELE filteret (alle sektioner)
        self._zi = None # udskydes til første kørsel

    # Filters the signal through the bandpass filter
    def process(self, x: np.ndarray) -> np.ndarray:
        # Input validering
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