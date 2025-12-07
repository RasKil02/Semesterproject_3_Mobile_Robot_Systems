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

proto = Protocol()

def choose_device(prompt):
    print(prompt)
    user_input = input().strip()

    if user_input == "":
        return None  # Use system default
    try:
        return int(user_input)
    except ValueError:
        print("Invalid input. Please enter a number.")
        return choose_device(prompt)

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
        sd.default.reset()

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")

    return cmd


def main(inputdevice, outputdevice):
    while True:
        NACK = 'A'
        ACK = 'B'
        restart_counter = 0

        # Send første kommando

        sd.default.device = (inputdevice, outputdevice)
        proto.play_dtmf_command_checksum(8000)
        
        while True:
            
            print("Waiting for possible NACK or ACK response\n")
            FeedbackCommand = readCommandDuration(8)
            print("Received:", FeedbackCommand)

            if FeedbackCommand == ACK:
                print("ACK received. Proceeding to next command\n")
                break

            elif FeedbackCommand == NACK:
                print("NACK received\n")
                print("Using output device ID:", outputdevice)
                sd.default.device = (inputdevice, outputdevice)
                if restart_counter == 2:
                    print("No valid ACK recieved after 3 attempts. Aborting...\n")
                    break
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1

            else:
                print("No valid feedback received → Resending command...\n")
                sd.default.device = (inputdevice, outputdevice)
                if restart_counter == 2:
                    print("No valid response after 3 attempts. Aborting...\n")
                    break
                proto.play_dtmf_command_checksum(8000, proto.command, True)
                restart_counter += 1    


if __name__ == "__main__":
    print("\n=== Audio Devices ===")
    print(sd.query_devices())

    input_dev = choose_device(
        "\nEnter INPUT device ID (microphone) or press Enter for default:"
    )
    output_dev = choose_device(
        "Enter OUTPUT device ID (speaker) or press Enter for default:"
    )

    # Build tuple for sounddevice
    current = sd.default.device
    sd.default.device = (
        input_dev if input_dev is not None else current[0],
        output_dev if output_dev is not None else current[1]
    )

    print(f"\nUsing devices: Input={sd.default.device[0]}  Output={sd.default.device[1]}")
    
    try:
        main(input_dev, output_dev)
    except KeyboardInterrupt:
        print("\n\nKeyboardInterrupt detected. Exiting gracefully...\n")
    finally:
        # Hvis du senere får globale ressourcer, kan du lukke dem her
        print("Cleanup complete. Goodbye!")
