from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
import rclpy

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
        command = "00000001" + "00000010"  # dest=2, supplies=2
        node.chooseRoute(command)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

def main():
    #runRobotWithRoutePlanner()
    testRobotMovement()

# Run the main function when this script is executed
if __name__ == '__main__':
    main()
