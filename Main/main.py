from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
from ProtocolSpeaker_connection.Protocol import Protocol
import rclpy
import time

def playCommand():
    protocol = Protocol()
    command = protocol.play_DTMF_command("0000")
    print("Command received:", command)
    # Here you would add code to process and execute the command

# Basic function to test robot movement
def testRobotMovement():
    # Initialize ROS 2
    rclpy.init()

    # Create ROS 2 node
    node = MoveTest() 

    try:
        node.run_sequence() # Execute movement sequence (planned in MoveTest)
    
    # Keyboard interrupt to stop the program (CTRL+C)
    except KeyboardInterrupt:
        pass

    # No matter what, ensure the robot stops and the node is properly destroyed
    finally:
        node.stop(0.3)
        node.destroy_node()
        rclpy.shutdown()

# Used to run the robot with route planning, based on protocol user input (until microphone is implemented)
def runRobotWithRoutePlanner():
    rclpy.init()
    node = RoutePlanner()
    try:
        # dest=1, supplies=0
        command1 = "000000" + "000000"
        node.chooseRoute(command1)
        time.sleep(3)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        
def readCommand():
    pass

def convertCommand(command: str) -> str:
    if len(command) % 2:
        raise ValueError("Længden skal være lige (par af cifre).")
    parts = []
    for i in range(0, len(command), 2):
        a, b = command[i], command[i+1]
        if a not in "01234567" or b not in "01234567":
            raise ValueError("Kun 0–7 er tilladt.")
        parts.append(f'"{format(int(a), "03b")}{format(int(b), "03b")}"')
    return " + ".join(parts)

def main():
    playCommand()
    readCommand()
    convertCommand()
    runRobotWithRoutePlanner()

# Run the main function when this script is executed
if __name__ == '__main__':
    main()
