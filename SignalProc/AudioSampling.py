import numpy as np
import matplotlib.pyplot as plt
import sounddevice as sd
import argparse
from scipy.io import wavfile

class AudioSampler:
    def __init__(self, duration, fs, output):
        self.duration = duration
        self.fs = fs
        self.output = output
        self.audio = None

    def record_audio(self, device=None):
        # Hvis ingen device angivet, find automatisk "USB Audio"
        if device is None:
            for i, d in enumerate(sd.query_devices()):
                if "USB Audio" in d['name'] and d['max_input_channels'] > 0:
                    device = i
                    break

        sd.default.device = (device, None)
        sd.default.samplerate = self.fs

        print(f"Recording from device {device} for {self.duration} seconds at {self.fs} Hz...")
        self.audio = sd.rec(int(self.duration * self.fs),
                            samplerate=self.fs, channels=1, dtype='float32')
        sd.wait()

        # niveau-diagnostik
        a = self.audio.squeeze().astype(float)
        peak = float(np.max(np.abs(a)))
        rms  = float(np.sqrt(np.mean(a*a)))
        print(f"[Audio] peak={peak:.6f}, rms={rms:.6f}")
        print("Recording complete.")
        return self.audio.flatten()

    def save_audio(self):
        import numpy as np
        from scipy.io import wavfile
        if self.audio is None:
            raise RuntimeError("Call record_audio() first.")
        a = self.audio.squeeze().astype('float32')
        peak = float(np.max(np.abs(a))) or 1.0
        a = a / peak   # KUN for at kunne høre den ved afspilning
        wavfile.write(self.output, self.fs, (a*32767).astype(np.int16))
        print(f"Audio saved to {self.output} (normalized)")

    def plot_waveform(self):
        if self.audio is None:
            raise ValueError("No audio recorded to plot.")
        plt.figure(figsize=(10, 4))
        plt.plot(np.linspace(0, len(self.audio) / self.fs, num=len(self.audio)), self.audio)
        plt.title("Audio Waveform")
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.grid()
        plt.show()

