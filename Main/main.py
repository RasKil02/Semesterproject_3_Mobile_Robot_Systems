from DriveSystem.NotUsed.MoveTest import MoveTest
from DriveSystem.RoutePlanner import RoutePlanner
from SignalProc.DTMFDetector import DTMFDetector
from SignalProc.DTMFDetector import DigitStabilizer
import rclpy
import time
import argparse

def testRobotMovement():
    """Simple manual test to verify robot motion via MoveTest node."""
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
    """Run the robot route planner based on the received command string."""
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
    """Record audio from microphone and detect DTMF digits."""
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
    print("\n--- Detected digits ---")
    print(digits if digits else "(none)")
    return digits


def convertCommand(command: str) -> str:
    """Convert intermediate digit command string to binary format for RoutePlanner."""
    if len(command) % 2:
        raise ValueError("Længden skal være lige (par af cifre).")

    parts = []
    for i in range(0, len(command), 2):
        a, b = command[i], command[i + 1]
        if a not in "01234567" or b not in "01234567":
            raise ValueError("Kun 0-7 er tilladt.")
        parts.append(f'{format(int(a), "03b")}{format(int(b), "03b")}')
    return "".join(parts)


if __name__ == "__main__":
    # Optional direct run: works if you ever want to test everything on one machine
    command = readCommand()
    converted = convertCommand(command)
    print("Converted command:", converted)
    runRobotWithRoutePlanner(converted)
