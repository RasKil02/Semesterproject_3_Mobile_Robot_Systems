from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
import rclpy
import time

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
        command1 = "00000000" + "00000000"
        node.chooseRoute(command1)
        time.sleep(3)

        # dest=2, supplies=1
        command2 = "00000001" + "00000001"
        node.chooseRoute(command2)
        time.sleep(3)

        # dest=3, supplies=1
        command3 = "00000010" + "00000001"
        node.chooseRoute(command3)
        time.sleep(3)

        # dest=4, supplies=2
        command4 = "00000011" + "00000010"
        node.chooseRoute(command4)
        time.sleep(3)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

    
def main():
    runRobotWithRoutePlanner()
    #testRobotMovement()

# Run the main function when this script is executed
if __name__ == '__main__':
    main()
