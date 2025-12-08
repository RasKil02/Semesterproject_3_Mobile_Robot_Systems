import serial
import time

class PicoMotorController:
    def __init__(self, port="/dev/serial/by-id/usb-MicroPython_*", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Allow Pico reconnect after open

    def send_supply_id(self, supply_id: int):
        msg = f"{supply_id}\n".encode()
        self.ser.write(msg)

        # Wait for ACK
        ack = self.ser.readline().decode().strip()
        return ack

    def close(self):
        self.ser.close()
