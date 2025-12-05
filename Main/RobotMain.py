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

#sd.default.device = (1, None)      # Use USB sound card for output
#sd.default.channels = 2            # Your USB device requires stereo
#sd.default.samplerate = 48000      # Safe working sample rate

from ProtocolSpeaker_connection.Protocol import Protocol
import rclpy

from DriveSystem.RoutePlanner import RoutePlanner
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse
import threading
import queue

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

    print("Listening for DTMF command (*#, then 6 digits)...")
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

    print("Listening for DTMF command (*#, then 6 digits)...")
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
    nack_command = "A"
    ack_command = "B"
    retransmit = False

    while True:
        command = readCommand()
        print("Received command:", command)

        # Check for retransmission
        if retransmit == True:
            if command[7] == seqNrDigit:
                    print("Expected new command but received old command. Sending ACK to request new command")
                    proto.play_DTMF_command(ack_command) # Plays ACK to request new command
                    continue

        # Dekod og check CRC
        try:
            cmd_no_prefix, bitstring, is_valid, remainder, checksum_digit, seqNrDigit = \
                proto.decode_and_check_crc(command)
        except ValueError:
            print("Error decoding command for checksu. One bit was not a number\n" )

        # Hvis checksum ikke er valid → send NACK
        if not is_valid:
            retransmit = False  # Forventer genafsendelse efter NACK
            time.sleep(2) # Giver tid til at computer sender kommando færdig.
            print("Checksum invalid → sending NACK")

            proto.play_DTMF_command(nack_command, 41100)
            continue  # Gå tilbage til starten af while-loopet for at vente på ny kommando
        
        if is_valid:
            proto.play_DTMF_command(ack_command, 41100)
            retransmit = True  # Forventer ny kommando efter ACK
                    
            # Konverter til bitstring som RoutePlanner tager som input
            bitstring = bitstring[0:12] 
            print("Checksum valid sender ACK")
            print("executing route planner with bitstring: " + bitstring) 
            runRobotWithRoutePlanner(bitstring)
            continue

    # Exit loop with keyboard interrupt
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            running = False
            print("Exiting program.")


if __name__ == "__main__":
    main()


