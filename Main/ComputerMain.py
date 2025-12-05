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
from ProtocolSpeaker_connection.Protocol import Protocol
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse

def readCommandDuration(duration, sampler):

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

    print("Listening for DTMF command (*#, then 5 digits)...")
    cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def main():

    while True:
        NACK = 'A'
        ACK = 'B'
        proto = Protocol()

        sampler = AudioSampler()

        # Send første kommando
        proto.play_dtmf_command_checksum(8000)

        while True:
            print("Waiting for possible NACK or ACK response\n")
            FeedbackCommand = readCommandDuration(10, sampler)
            print("Received:", FeedbackCommand)

            # --- ACK → exit loop ---
            if FeedbackCommand == ACK:
                print("ACK received. Proceeding to next command\n")
                break

            # --- NACK → resend with NACK flag ---
            elif FeedbackCommand == NACK:
                print("NACK received. Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command, True)

            # --- None, tom streng eller noget andet → resend normal ---
            else:
                print("No valid feedback received → Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command)

        sd.stop()


if __name__ == "__main__":
    main()