import numpy as np
from scipy.signal import butter, lfilter

class BandPassFilter:
    def __init__(self, lowcut, highcut, fs, order=5):
        self.lowcut = lowcut
        self.highcut = highcut
        self.fs = fs
        self.order = order
        self.b, self.a = self.butter_bandpass()

    def butter_bandpass(self):
        nyq = 0.5 * self.fs
        low = self.lowcut / nyq
        high = self.highcut / nyq
        b, a = butter(self.order, [low, high], btype='band')
        return b, a

    def apply_filter(self, data):
        y = lfilter(self.b, self.a, data)
        return y