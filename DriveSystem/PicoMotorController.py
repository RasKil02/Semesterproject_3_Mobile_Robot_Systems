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

    def _install_motor_code(self):
        """Load PWM motor driver code into Pico RAM."""
        code = """
        from machine import Pin, PWM

        motor1 = PWM(Pin(15))
        motor2 = PWM(Pin(16))

        motor1.freq(1000)
        motor2.freq(1000)

        def set_m1(pct):
            motor1.duty_u16(int(65535 * max(0, min(1, pct))))

        def set_m2(pct):
            motor2.duty_u16(int(65535 * max(0, min(1, pct))))

        print("Pico motor PWM ready.")
        """
        print(self._send_raw(code))

    # -------------------------
    # Public control methods
    # -------------------------

    def set_motor1(self, percent: float):
        """Set Motor 1 speed from 0.0 to 1.0."""
        cmd = f"set_m1({percent})"
        self._send_raw(cmd)

    def set_motor2(self, percent: float):
        """Set Motor 2 speed from 0.0 to 1.0."""
        cmd = f"set_m2({percent})"
        self._send_raw(cmd)

    def stop_all(self):
        """Stop both motors."""
        self._send_raw("set_m1(0); set_m2(0)")

    def close(self):
        """Close serial connection."""
        self.stop_all()
        self.ser.close()

    # -------------------------
    # Optional helper
    # -------------------------

    def sweep_test(self):
        """Quick test of both motors."""
        for pct in [0, 0.25, 0.5, 0.75, 1.0, 0]:
            print(f"Sweeping to {pct*100:.0f}%")
            self.set_motor1(pct)
            self.set_motor2(pct)
            time.sleep(1)
