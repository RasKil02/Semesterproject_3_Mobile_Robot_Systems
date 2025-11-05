import socket
from Main.main import readCommand, convertCommand


def sendCommandToRobot(command: str):
    """Send the converted DTMF command string to the robot over TCP."""
    HOST = "172.20.10.2"   
    PORT = 5000 

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"Connecting to robot at {HOST}:{PORT} ...")
            s.connect((HOST, PORT))
            s.sendall(command.encode('utf-8'))
            print(f"Sent command to robot: {command}")
    except ConnectionRefusedError:
        print("Could not connect to the robot. Is receive_and_run.py running on it?")
    except Exception as e:
        print(f"Error sending command: {e}")


def main():
    """Record, detect, convert, and send."""
    print("Listening for DTMF tones...")
    command = readCommand()                 
    converted_command = convertCommand(command) 
    print(f"Detected command: {command}")
    print(f"Converted command: {converted_command}")

    sendCommandToRobot(converted_command)
    print("Done. Waiting for the robot to execute the route.")


if __name__ == "__main__":
    main()