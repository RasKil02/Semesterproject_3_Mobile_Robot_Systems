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
from DriveSystem.PicoMotorController import PicoMotorController
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
from SignalProc.AudioSampling import AudioSampler
import time
import argparse

# Function
def testRobotMovement():
    rclpy.init()
    node = MoveTest()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop(0.3)
        node.destroy_node()
        rclpy.shutdown()

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


# Converts digit into 3 bit binary number, pairs of digits to 6 bit binary numbers
def convertCommand(command: str) -> str:
    if len(command) % 2:
        raise ValueError("Længden skal være lige (par af cifre).")

    parts = []
    for i in range(0, len(command), 2):
        a, b = command[i], command[i + 1]
        if a not in "01234567" or b not in "01234567":
            raise ValueError("Kun 0-7 er tilladt.")
        parts.append(f'{format(int(a), "03b")}{format(int(b), "03b")}')
    return "".join(parts) # Laver en 12 bit streng til samlet streng af 6 bit par

def readUntilDetected():
    commandRead = False
    
    while commandRead == False:
        command = readCommand()
        if len(command) == 7:
            commandRead = True
            return command
        else:
            print("Ugyldig kommando modtaget. Prøv igen.")

def isValidCommand(command: str, proto: Protocol):
    # Gem den sidste DTMF tone til checksum validering
    checksumDigit = (command[6])
    command = command[2:7]  # Fjern de første 2 toner for at lave convertCommand med de næste 4 toner
    print("Command for checksum: " + command)

    print ("Checksum DTMF tone: " + checksumDigit)
    received_bitstring = proto.decimal_string_to_3bit_binary_string(command)
    print("Converted command to bits:", received_bitstring)
    remainder, is_valid = proto.Check_CRC(received_bitstring)
    print("Remainder after CRC:", remainder)
    print("Is CRC valid?", is_valid)
    return received_bitstring, is_valid

if __name__ == "__main__":
    
    running = True
    
    while running:
        command = readCommand()
    
        proto = Protocol()
        received_bitstring, is_valid = isValidCommand(command, proto) # Check if command is valid - Checksum validation
        print("Received bitstring:", received_bitstring)

        if is_valid:
            print("Command is valid. Executing route planner...")  
            runRobotWithRoutePlanner(received_bitstring)
        else:
            print("Invalid command received. Aborting operation.")
            
        # Exit loop with keyboard interrupt
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            running = False
            print("Exiting program.")
            
