from Protocol import protocol
import numpy as np
import sounddevice as sd

def main():
    my_protocol = protocol("MyProtocol")
    
    my_protocol.set_command()
    my_protocol.print_command()
    my_protocol.translateCommandToDTMFfreq(f"{my_protocol.start}{my_protocol.roomAddress}{my_protocol.supplyAdress}{my_protocol.stop}")
    my_protocol.print_DTMF_command()
    my_protocol.play_DTMF_command(f"{my_protocol.start}{my_protocol.roomAddress}{my_protocol.supplyAdress}{my_protocol.stop}")

if __name__ == "__main__":
    main()
