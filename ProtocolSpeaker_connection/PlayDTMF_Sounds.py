import numpy as np
import sounddevice as sd
from Protocol import Protocol
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer

def readCommandDuration(duration):
    # Du kan fjerne argparse, hvis du ikke længere vil bruge kommandolinjeargumenter
    fs = 8000
    out = "output.wav"
    block_ms = 30.0
    hop_ms = 7.5

    detector = DTMFDetector(
        fs=fs,
        block_ms=block_ms,
        hop_ms=hop_ms,
        lowcut=620, highcut=1700, bp_order=4,
        min_db=-20, sep_db=5, dom_db=4, snr_db=8,
        twist_pos_db=+4, twist_neg_db=-8
    )

    stab = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    digits = detector.record_and_detect(duration, out, stabilizer=stab)
    print("\n--- Detected digits ---")
    print(digits if digits else "(none)")
    return digits # Return the detected command string (DTMF toner 1234 giver en string "1234")


def main():
    while True:
        proto = Protocol()
        proto.play_dtmf_command_checksum()

        while True:
            NACKCommand = readCommandDuration(10.0)
            print("Received NACK command:", NACKCommand)
            if (NACKCommand == None):
                print("No command received, exiting.")
                break

            if (NACKCommand != None):
                proto.play_dtmf_command_checksum(proto.command)
        
        sd.stop()

if __name__ == "__main__":
    main()
