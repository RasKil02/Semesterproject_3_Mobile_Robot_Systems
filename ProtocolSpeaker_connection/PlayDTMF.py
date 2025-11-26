import sys
import os

# Find mappen hvor dette script kører fra (ProtocolSpeaker_connection)
current_dir = os.path.dirname(__file__)  
# Gå et niveau op til projektroden (Semesterproject_3_Mobile_Robot_Systems)
project_root = os.path.abspath(os.path.join(current_dir, '..'))  
# Tilføj projektroden til sys.path så du kan importere søskende mapper som DriveSystem
sys.path.append(project_root)

import numpy as np
import sounddevice as sd
from Protocol import Protocol
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse

def readCommandDuration(duration):
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--fs", type=int, default=44100)
    ap.add_argument("--block_ms", type=float, default=30.0)
    ap.add_argument("--hop_ms",   type=float, default=7.5)
    args = ap.parse_args()

    # --- Create detector ---
    detector = DTMFDetector(
        fs=args.fs,
        block_ms=args.block_ms,
        hop_ms=args.hop_ms,
        lowcut=620, highcut=1700, bp_order=4,
        min_db=-20, sep_db=5, dom_db=4, snr_db=8,
        twist_pos_db=+4, twist_neg_db=-8
    )

    # --- Stabilizer ---
    stabilizer = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    # --- Audio sampler (streaming) ---
    sampler = AudioSampler(fs=args.fs)

    print(f"Listening for DTMF command for max {duration} seconds (*#, then 5 digits)...")
    cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def main():
    while True:
        proto = Protocol()
        proto.play_dtmf_command_checksum()

        """""
        while True:
            print("Waiting for possible NACK response\n")
            NACKCommand = readCommandDuration(10)
            print("Received NACK command:", NACKCommand)

            if (not NACKCommand):
                print("No command NACK received.")
                print("Continuing to next command\n")
                break

            if (NACKCommand != None):
                proto.play_dtmf_command_checksum(proto.command)
        """""
        
        sd.stop()

if __name__ == "__main__":
    main()
