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

        # Install motor PWM code on Pico
        self._install_motor_code()

    def _send_raw(self, code: str):
        """Send a chunk of MicroPython code to the Pico and execute it."""
        self.ser.write(b'\x01')  # Enter RAW REPL (CTRL-A)
        time.sleep(0.03)

        cleaned = textwrap.dedent(code).strip() + "\r"
        self.ser.write(cleaned.encode("utf-8"))
        self.ser.write(b'\x04')  # Execute (CTRL-D)

        time.sleep(0.05)
        return self.ser.read(5000).decode(errors="ignore")

    def stop_all(self):
        """Stop both motors."""
        self._send_raw("set_m1(0); set_m2(0)")

    def close(self):
        """Close serial connection."""
        self.stop_all()
        self.ser.close()