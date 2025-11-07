import argparse
import numpy as np
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.AudioSampling import AudioSampler

def readCommand():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--fs", type=int, default=8000)
    parser.add_argument("--out", type=str, default="output.wav")
    parser.add_argument("--block_ms", type=float, default=30.0)
    parser.add_argument("--hop_ms", type=float, default=7.5)
    args = parser.parse_args()

    detector = DTMFDetector(
        fs=args.fs,
        block_ms=args.block_ms,
        hop_ms=args.hop_ms,
        lowcut=620,
        highcut=1700,
        bp_order=4
    )

    # Record audio
    sampler = AudioSampler(args.duration, args.fs, args.out)
    audio = sampler.record_audio()
    sampler.save_audio()

    # Simple mic check
    if np.max(np.abs(audio)) < 0.001:
        print("No significant sound detected — check your microphone.")
        return "(no audio)"

    # Analyze if sound was detected
    digits = detector.analyze(audio)
    print("\n--- Detected digits ---")
    print(digits if digits else "(none)")
    return digits

if __name__ == "__main__":
    readCommand()   