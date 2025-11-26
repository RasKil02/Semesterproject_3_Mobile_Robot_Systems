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
import rclpy
from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse
import threading
import queue

#import serial

proto = Protocol()
newCommandEvent = threading.Event()
control_queue = queue.Queue()

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

def readCommand():

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

    # --- Audio sampler (streaming) ---
    sampler = AudioSampler()

    print("Listening for DTMF command (*#, then 5 digits)...")
    cmd = detector.stream_and_detect(stabilizer, sampler)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def runRobotWithRoutePlanner(command: str):
    rclpy.init()
    node = RoutePlanner()
    try:
        node.chooseRoute(command)
        time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

def main():
    while True:
        command = readCommand()

        print("Received command:", command)

        # Dekod og check CRC
        try:
            cmd_no_prefix, bitstring, is_valid, remainder, checksum_digit = \
                proto.decode_and_check_crc(command)
        except ValueError:
            print("Error decoding command for checksu. One bit was not a number\n" )
            is_valid = False

        # Hvis checksum ikke er valid → send NACK
        while not is_valid:
            time.sleep(5) # Giver tid til at computer sender kommando færdig.
            print("Checksum invalid → sending NACK")

            nack_command = "A"
            proto.play_DTMF_command(nack_command, duration=0.5)

            # Vent op til 10 sek. på en ny kommando
            RestransmittedCommand = readCommandDuration(10)

            if not RestransmittedCommand:
                print("Timeout → No command received → sending NACK again")
                continue

            # Ny kommando modtaget
            if RestransmittedCommand is not None:
                print("New command received:", RestransmittedCommand)

            # Tjek den nye kommando
                try:
                    cmd_no_prefix, bitstring, is_valid, remainder, checksum_digit = \
                        proto.decode_and_check_crc(RestransmittedCommand)
                    
                except ValueError:
                    print("Error decoding command for checksu. One bit was not a number\n" )
                    is_valid = False
                except IndexError:
                    print("Error decoding command: Command too short (index out of range)\n")
                    is_valid = False
            
            if (is_valid):
                print("Checksum valid efter NACK → fortsætter")
                break
            else:
                continue
                
        # Konverter til bitstring som RoutePlanner tager som input
        bitstring = bitstring[0:12] 

        print("Checksum valid → executing route planner with bitstring: " + bitstring) 
        runRobotWithRoutePlanner(bitstring)

    # Exit loop with keyboard interrupt
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            running = False
            print("Exiting program.")


if __name__ == "__main__":
    main()


