import serial
import time
import textwrap

class PicoMotorController:
    """
    Controls two DC motors on a Raspberry Pi Pico using PWM (GPIO 15 and 16).
    Communicates via the Pico USB raw-REPL interface.
    """

    def __init__(self, port="/dev/ttyACM0", baud=115200):
        self.port = port
        self.baud = baud

        # Open serial connection to Pico
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # allow Pico reboot

    def send_supply_id(self, supply_id: int):
        self.ser.write(str(supply_id).encode())


    def stop_all(self):
        """Stop both motors."""
        self._send_raw("set_m1(0); set_m2(0)")

    def close(self):
        """Close serial connection."""
        self.stop_all()
        self.ser.close()