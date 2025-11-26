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
import rclpy
from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse
import threading
#import serial


proto = Protocol()
newCommandEvent = threading.Event()
route = RoutePlanner()

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


def listener_thread():
    global command
    while True:
        command = readCommand()
        print("Received:", command)

        if command:
            newCommandEvent.set()  # Signalér til main at en ny kommando er modtaget

t1 = threading.Thread(target=listener_thread, daemon=True)

def main():
    global command

    while True:

        # Vent på en ny kommando fra thread
        newCommandEvent.wait()
        newCommandEvent.clear()

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
    t1.start()
    main()


