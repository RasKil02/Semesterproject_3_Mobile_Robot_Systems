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

sd.default.device = (0, 1)
sd.default.channels = (2, 1)    # 2 output channels (stereo), 1 input channel (mono)
sd.default.samplerate = 48000

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

def readCommandDuration(duration, sampler):

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

    print("Listening for DTMF command (*#, then 6 digits)...")
    cmd = detector.stream_and_detect_duration(stabilizer, sampler, duration)

    print("\n--- Detected command ---")
    print(cmd if cmd else "(none)")
    return cmd

def readCommand(sampler):

    # --- Create detector ---
    detector = DTMFDetector()

    # --- Stabilizer ---
    stabilizer = DigitStabilizer()

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

    sampler = AudioSampler()

    while True:
        command = readCommand(sampler)
        print("Received command:", command)

        if retransmit:
            if command[7] == seqNrDigit:
                print("Expected new command but received old command. Sending ACK")
                sampler.close()
                sd.stop()
                proto.play_DTMF_command(ack_command, 48000)
                sampler = AudioSampler()
                continue

        cmd_no_prefix, bitstring, is_valid, remainder, checksum_digit, seqNrDigit = \
            proto.decode_and_check_crc(command)

        if not is_valid:
            print("Checksum invalid → sending NACK")
            sampler.close()
            sd.stop()
            proto.play_DTMF_command(nack_command, 48000)
            sampler = AudioSampler()
            continue

        print("Checksum valid → sending ACK")
        sampler.close()
        sd.stop()
        proto.play_DTMF_command(ack_command, 48000)

        retransmit = True

        bitstring = bitstring[:12]
        print("executing route planner with bitstring:", bitstring)
        runRobotWithRoutePlanner(bitstring)

        sampler = AudioSampler()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting program.")


