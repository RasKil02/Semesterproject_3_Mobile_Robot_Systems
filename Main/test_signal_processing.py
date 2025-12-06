import sys
import os

# Find mappen hvor dette script kører fra (ProtocolSpeaker_connection)
current_dir = os.path.dirname(__file__)  
project_root = os.path.abspath(os.path.join(current_dir, '..'))  
sys.path.append(project_root)

import argparse
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler


def readCommand():

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

    # --- Audio sampler (streaming) ---
    sampler = AudioSampler()

    print("Listening for DTMF command (*#, then 6 digits)...")
    cmd = detector.stream_and_detect(stabilizer, sampler, True)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd


if __name__ == "__main__":
    readCommand()
