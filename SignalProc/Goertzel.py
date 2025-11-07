import numpy as np
import math
from typing import Iterable, Dict

class GoertzelAlgorithm:
    def __init__(self, fs: int, block: int, target_freqs: Iterable[float]):
        self.fs = int(fs)
        self.N = int(block)
        self.target_freqs = list(target_freqs)
        self.coefficients = self._compute_coefficients()  # dict: f -> coeff

    def _compute_coefficients(self) -> Dict[float, float]:
        coeffs: Dict[float, float] = {}
        for f in self.target_freqs:
            omega = 2.0 * math.pi * f / self.fs
            coeffs[f] = 2.0 * math.cos(omega)
        return coeffs

    def process(self, signal: np.ndarray) -> Dict[float, float]:
        x = np.asarray(signal, dtype=np.float64).squeeze()
        if x.ndim != 1:
            raise ValueError("GoertzelAlgorithm.process forventer 1D signal.")
        if len(x) != self.N:
            raise ValueError(f"Bloklængde mismatch: len(x)={len(x)} men N={self.N}.")

        # DC-fjernelse (vindue laves i main; ingen vindue her)
        x = x - float(np.mean(x))

        results: Dict[float, float] = {}
        for f in self.target_freqs:
            coeff = self.coefficients[f]
            s_prev = 0.0
            s_prev2 = 0.0
            for xn in x:
                s = xn + coeff * s_prev - s_prev2
                s_prev2, s_prev = s_prev, s
            # Stabil standard-power
            power = s_prev*s_prev + s_prev2*s_prev2 - coeff*s_prev*s_prev2
            results[f] = power
        return results
