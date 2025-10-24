from DriveSystem import MoveTest
from CommunicationProtocol import Protocol
import numpy as np
import sounddevice as sd

def main():

    #  Example of how to instantiate protocol class
    #my_protocol = protocol("MyProtocol")
    
    #my_protocol.set_command()
    #my_protocol.print_command()
    #my_protocol.translateCommandToDTMFfreq(f"{my_protocol.start}{my_protocol.roomAddress}{my_protocol.supplyAdress}{my_protocol.stop}")
    #my_protocol.print_DTMF_command()
    #my_protocol.play_DTMF_command(f"{my_protocol.start}{my_protocol.roomAddress}{my_protocol.supplyAdress}{my_protocol.stop}")
    #





    # Example of how to use differential drive class (Get the robot to move)
    pass

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

if __name__ == '__main__':
    main()

