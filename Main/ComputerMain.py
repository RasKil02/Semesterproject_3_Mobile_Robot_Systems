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

def readCommandDuration(duration):

    detector = DTMFDetector()
    stabilizer = DigitStabilizer()
    sampler = AudioSampler()

    print("Listening for DTMF command (*#, then 5 digits)...")

    try:
        cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)
    finally:
        # ALWAYS executed
        sampler.close()
        sd.stop()

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")

    return cmd


def main():

    while True:
        NACK = 'A'
        ACK = 'B'
        proto = Protocol()
        restart_counter = 0

        # Send første kommando
        proto.play_dtmf_command_checksum(8000)
        

        while True:
            
            if restart_counter >= 4:
                print("No valid ACK received after 4 attempts. Aborting...\n")
                break
            
            print("Waiting for possible NACK or ACK response\n")
            FeedbackCommand = readCommandDuration(15)
            print("Received:", FeedbackCommand)

            # --- ACK → exit loop ---
            if FeedbackCommand == ACK:
                print("ACK received. Proceeding to next command\n")
                break

            # --- NACK → resend with NACK flag ---
            elif FeedbackCommand == NACK:
                print("NACK received. Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1

            # --- None, tom streng eller noget andet → resend normal ---
            else:
                print("No valid feedback received → Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1    


if __name__ == "__main__":
    main()