import numpy as np
import math
from typing import Iterable, Dict, List

class GoertzelAlgorithm:
    def __init__(self, fs: int, block: int, target_freqs: Iterable[float]):
        self.fs = fs
        self.N = block
        self.target_freqs = list(target_freqs)
        self.coefficients = self._compute_coefficients()

    def _compute_coefficients(self) -> Dict[float, float]:
        coeffs: Dict[float, float] = {}
        for f in self.target_freqs:
            k = int(round(self.N * f / self.fs))
            omega = (2.0 * math.pi * k) / self.N
            coeffs[f] = 2.0 * math.cos(omega)
        return coeffs

    def process(self, signal: np.ndarray) -> Dict[float, float]:
        """Returnér power for hver target-frekvens på en blok (længde N)."""
        x = signal.astype(float)
        results: Dict[float, float] = {}
        for f, coeff in self.coefficients.items():
            s1 = 0.0; s2 = 0.0
            for xn in x:
                s = xn + coeff * s1 - s2
                s2, s1 = s1, s
            power = s2*s2 + s1*s1 - coeff*s1*s2
            results[f] = power / (len(x)**2)  # let normalisering
        return results
    
    
