import numpy as np
import sounddevice as sd
from Protocol import Protocol

def main():
    proto = Protocol()
    proto.play_dtmf_command_checksum()
    sd.stop()

if __name__ == "__main__":
    main()
