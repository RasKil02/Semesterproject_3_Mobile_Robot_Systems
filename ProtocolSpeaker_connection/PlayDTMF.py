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

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

    # --- Audio sampler (streaming) ---
    sampler = AudioSampler()

    print("Listening for DTMF command (*#, then 5 digits)...")
    cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def main():
    while True:
        proto = Protocol()
        proto.play_dtmf_command_checksum()

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
        
        sd.stop()

if __name__ == "__main__":
    main()
