import numpy as np
import math
from typing import Iterable, Dict

class GoertzelAlgorithm:
    def __init__(self, fs: int, target_freqs: Iterable[float]):
        self.fs = fs
        self.target_freqs = target_freqs
        self.coefficients = self._compute_coefficients()

    def _compute_coefficients(self) -> Dict[float, float]:
        coefficients = {}
        for freq in self.target_freqs:
            k = int(0.5 + (self.fs * freq) / self.fs)
            omega = (2.0 * math.pi * k) / self.fs
            coefficients[freq] = 2.0 * math.cos(omega)
        return coefficients

    def process(self, signal: np.ndarray) -> Dict[float, float]: # blokvis beregning
        results = {}
        for freq, coeff in self.coefficients.items():
            s_prev = 0.0
            s_prev2 = 0.0
            for sample in signal:
                s = sample + coeff * s_prev - s_prev2
                s_prev2 = s_prev
                s_prev = s
            power = s_prev2**2 + s_prev**2 - coeff * s_prev * s_prev2
            results[freq] = power
        return results