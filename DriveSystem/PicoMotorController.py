import serial
import time

class PicoMotorController:
    """
    Sends simple one-byte commands (e.g., b"1") to the Pico via USB serial.
    """

    def __init__(self, port="/dev/ttyACM0", baud=115200):
        self.port = port
        self.baud = baud

        # Open serial connection to Pico USB
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # allow Pico to reboot when serial opens

    def send_supply_id(self, supply_id: int):
        # Send the ID as a single byte/character
        self.ser.write(str(supply_id).encode())

    def close(self):
        self.ser.close()
