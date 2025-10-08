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

    def record_audio(self):
        print(f"Recording for {self.duration} seconds...")
        self.audio = sd.rec(int(self.duration * self.fs), samplerate=self.fs, channels=1)
        sd.wait()
        print("Recording complete.")
        return self.audio.flatten()

    def save_audio(self):
        wavfile.write(self.output, self.fs, self.audio)
        print(f"Audio saved to {self.output}")

    def plot_waveform(self):
        plt.figure(figsize=(10, 4))
        plt.plot(np.linspace(0, len(self.audio) / self.fs, num=len(self.audio)), self.audio)
        plt.title("Audio Waveform")
        plt.xlabel("Time [s]")
        plt.ylabel("Amplitude")
        plt.grid()
        plt.show()

def main():
    parser = argparse.ArgumentParser(description="Audio Sampler")
    parser.add_argument("--duration", type=int, default=5, help="Duration of recording in seconds")
    parser.add_argument("--fs", type=int, default=44100, help="Sampling frequency")
    parser.add_argument("--output", type=str, default="output.wav", help="Output WAV file name")
    args = parser.parse_args()

    sampler = AudioSampler(args.duration, args.fs, args.output)
    sampler.record_audio()
    sampler.save_audio()
    sampler.plot_waveform()

if __name__ == "__main__":
    main()