import socket
from Main.main import runRobotWithRoutePlanner


def main():
    HOST = "0.0.0.0"   # Listen on all available network interfaces
    PORT = 5000        # Must match the port used in listen_and_send.py

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Listening for commands on port {PORT}...")

        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                data = conn.recv(1024)
                if not data:
                    print("No data received.")
                    continue

                command = data.decode('utf-8').strip()
                print(f"Received command: {command}")

                try:
                    runRobotWithRoutePlanner(command)
                    print("Command execution complete.")
                except Exception as e:
                    print(f"Error running robot route: {e}")


if __name__ == "__main__":
    main()
