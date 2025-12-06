import sys
import os

# Find mappen hvor dette script kører fra (ProtocolSpeaker_connection)
current_dir = os.path.dirname(__file__)  
project_root = os.path.abspath(os.path.join(current_dir, '..'))  
sys.path.append(project_root)

import numpy as np
import sounddevice as sd
from ProtocolSpeaker_connection.Protocol import Protocol
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse

def print_output_device():
    dev = sd.default.device
    output_id = dev[1]  # (input_id, output_id)
    device_info = sd.query_devices(output_id)
    print(f"Using audio output device: {device_info['name']} (ID {output_id})")


def readCommandDuration(duration):

    detector = DTMFDetector()
    stabilizer = DigitStabilizer()
    sampler = AudioSampler()

    print("Listening for DTMF command (*#, then 6 digits)...")

    try:
        cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)
    finally:
        # ALWAYS executed
        sampler.close()

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")

    return cmd


def main():

    print_output_device()

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

            if FeedbackCommand == ACK:
                print("ACK received. Proceeding to next command\n")
                break

            elif FeedbackCommand == NACK:
                print("NACK received. Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1

            else:
                print("No valid feedback received → Resending command...\n")
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1    


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nKeyboardInterrupt detected. Exiting gracefully...\n")
    finally:
        # Hvis du senere får globale ressourcer, kan du lukke dem her
        print("Cleanup complete. Goodbye!")
