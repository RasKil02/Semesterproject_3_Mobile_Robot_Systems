from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
import rclpy
import time
import argparse

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

# Plays a DTMF command using the Protocol class
def playCommand():
    protocol = Protocol()
    command = protocol.play_DTMF_command("0000")
    print("Command played:", command)
    # Here you would add code to process and execute the command

# Used to run the robot with route planning, based on protocol user input (until microphone is implemented)
def runRobotWithRoutePlanner(command: str):
    # allow your own CLI args if you ever add some later
    parser = argparse.ArgumentParser(add_help=False)
    args, unknown = parser.parse_known_args()

    # let rclpy handle any ROS2 args such as --ros-args -p rate_hz:=40.0
    rclpy.init(args=unknown)

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

# Reads DTMF sounds and converts to intermidiate command string ex. "0000"
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

    # injicér stabilizer (hold/miss/gap kan du stadig tune frit)
    stab = DigitStabilizer(hold_ms=20, miss_ms=20, gap_ms=55)

    # optag og detektér i ét hug:
    digits = detector.record_and_detect(args.duration, args.out, stabilizer=stab)
    print("\n--- Detected digits ---")
    print(digits if digits else "(none)")
    return digits

# Converts intermidiate command string to binary string for route planner
def convertCommand(command: str) -> str:
    if len(command) % 2:
        raise ValueError("Længden skal være lige (par af cifre).")
    parts = []
    for i in range(0, len(command), 2):
        a, b = command[i], command[i+1]
        if a not in "01234567" or b not in "01234567":
            raise ValueError("Kun 0-7 er tilladt.")
        parts.append(f'"{format(int(a), "03b")}{format(int(b), "03b")}"')
    return " + ".join(parts)

def main():
    command = readCommand() # Reads DTMF sounds and converts to intermidiate command string ex. "0000"
    
    convertedCommand = convertCommand(command) # Converts intermidiate command string to binary string for route planner
    print(convertedCommand)
    
    runRobotWithRoutePlanner(convertedCommand) # Executes route planner with converted command

# Run the main function when this script is executed
if __name__ == '__main__':
    main()
