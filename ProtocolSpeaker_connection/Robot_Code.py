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
import time
import argparse
#import serial


proto = Protocol()
#route = RoutePlanner()


def readCommand():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--fs", type=int, default=8000)
    ap.add_argument("--out", type=str, default="output.wav")
    ap.add_argument("--block_ms", type=float, default=30.0)
    ap.add_argument("--hop_ms",   type=float, default=7.5)
    args = ap.parse_args()

    detector = DTMFDetector(
        fs=args.fs,
        block_ms=args.block_ms,
        hop_ms=args.hop_ms,
        lowcut=620, highcut=1700, bp_order=4,
        min_db=-20, sep_db=5, dom_db=4, snr_db=8,
        twist_pos_db=+4, twist_neg_db=-8
    )

    stab = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    digits = detector.record_and_detect(args.duration, args.out, stabilizer=stab)
    print("\n--- Detected digi        # Test robot movementts ---")
    print(digits if digits else "(none)")
    return digits # Return the detected command string (DTMF toner 1234 giver en string "1234")

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
    return "".join(parts) # Laver en 12 bit samlet streng af 6 bit par


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

    command = readCommand()
    print ("Received command: " + command)

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

    if not is_valid:
        print("Checksum invalid, sending NACK DTMF tone back to host computer.")
        # Send NACK DTMF tone back to host computer
        nack_command = "A"  # Assuming '9' represents NACK
        
        while True:
            proto.play_DTMF_command(nack_command)
            command = readCommandDuration(10)  # Wait for new command with timeout

            if command is not None:
                print("Received command after NACK: " + command)
                command = command[2:7]  # Fjern de første 2 toner for at lave convertCommand med de næste 4 toner
                print("Command for checksum: " + command)

                received_bitstring = proto.decimal_string_to_3bit_binary_string(command)
                print("Converted command to bits:", received_bitstring)
                remainder, is_valid = proto.Check_CRC(received_bitstring)
                print("Remainder after CRC:", remainder)
                print("Is CRC valid?", is_valid)

                if is_valid:
                    print("Checksum valid, proceeding with route planning.")
                    runRobotWithRoutePlanner(received_bitstring)
                    break
                else:
                    print("Checksum still invalid, waiting for new command.")

                if command is None:
                    print("Timeout reached, no command received.")


    roomadress = command[2:4]
    roomadress = int(roomadress)
    print("Room address: " + str(roomadress))


    #supplyroom = command[4:6]
    #print("Supply command: " + supplyroom)

    #supplyroom = int(supplyroom)
    #print("Supply room number: " + str(supplyroom))

    #route.send_data(supplyroom)

    







    # Fjern den sidste DTMF tone for at lave convertCommand med de første 4 toner

    # Skal have en ACK fra PC programmet
    # Hvis NACK så: skal bede om ny kommando eller bare vente og optage igen

    # Hvis ACD så:  


    #converted = convertCommand(command)
    #print("Converted command:", converted)
    #runRobotWithRoutePlanner(converted)

if __name__ == "__main__":
    main()


