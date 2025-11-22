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
import threading

proto = Protocol()
nack_event = threading.Event()

def readCommand():
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

    print("Listening for DTMF command (*#, then 5 digits)...")
    cmd = detector.stream_and_detect(stabilizer, sampler)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def listener_thread():
    while True:
        cmd = readCommand()
        print("Received NACK:", cmd)

        if cmd:

            nack_event.set()  # Signalér til main at NACK blev modtaget
            # Hvis robotten sender NACK → send kommando igen
            proto.play_dtmf_command_checksum(proto.command)


t1 = threading.Thread(target=listener_thread, daemon=True)


def main():
    while True:
        proto.play_dtmf_command_checksum()

        print("waiting 10 seconds for possible NACK response...")
        time.sleep(10)

        if nack_event.is_set():
            print("NACK received → extending wait by 10 seconds to send command again.")
            time.sleep(10)
            nack_event.clear()  # Reset event for next iteration
        # Code should be insertet that again time.sleep

        print("No NACK received, ready for next command.")

        sd.stop()

if __name__ == "__main__":
    t1.start()
    main()

