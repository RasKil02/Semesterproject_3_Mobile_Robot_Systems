import numpy as np
import math
from typing import Iterable, Dict

class GoertzelAlgorithm:
    def __init__(self, fs: int, block: int, target_freqs: Iterable[float]):
        # Goertzel uses fs (sampling frequency), block size N, and target frequencies
        self.fs = int(fs)
        self.N = int(block)
        self.target_freqs = list(target_freqs)
        
        # Precompute coefficients for each target frequency using compute coefficients method
        self.coefficients = self._compute_coefficients()  # dictionary: frequency and a matching coeff

    def _compute_coefficients(self) -> Dict[float, float]:
        coeffs: Dict[float, float] = {} # Create empty dictionary
        
        # Loops through each target frequency to calculate its coefficient
        for f in self.target_freqs:
            omega = 2.0 * math.pi * f / self.fs
            coeffs[f] = 2.0 * math.cos(omega)
        return coeffs

    def process(self, signal: np.ndarray) -> Dict[float, float]:
        """Returnér power for hver target-frekvens på en blok (længde N)."""
        
        samples = np.asarray(signal, dtype=np.float64).squeeze() # laver 1 dimensional array
        
        if samples.ndim != 1:
            raise ValueError("GoertzelAlgorithm.process forventer 1D signal.")
        if len(samples) != self.N:
            raise ValueError(f"Bloklængde mismatch: len(x)={len(samples)} men N={self.N}.")

        # DC offset-fjernelse - signal osciller omkring 0
        samples = samples - float(np.mean(samples))

        # Create empty dictionary to store results
        results: Dict[float, float] = {}
        
        for f in self.target_freqs:
            coeff = self.coefficients[f]
            s_prev = 0.0
            s_prev2 = 0.0
            for xn in samples:
                s = xn + coeff * s_prev - s_prev2
                s_prev2, s_prev = s_prev, s
            # Stabil standard-power
            power = s_prev*s_prev + s_prev2*s_prev2 - coeff*s_prev*s_prev2
            results[f] = power
        return results
