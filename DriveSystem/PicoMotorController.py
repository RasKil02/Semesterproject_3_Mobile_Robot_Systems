import serial
import time

class PicoMotorController:
    def __init__(self, port="/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6613008e37c6f34-if00", baud=115200):
        self.ser = serial.Serial(
            port,
            baud,
            timeout=1,
            write_timeout=1,
            dsrdtr=False,
            rtscts=False
        )

        # Ensure DTR stays LOW — critical for MicroPython USB CDC
        self.ser.dtr = False
        self.ser.rts = False

        time.sleep(0.5)  # allow USB stack to settle

    def send_supply_id(self, supply_id: int):
        msg = f"{supply_id}\n".encode()
        self.ser.write(msg)

        ack = self.ser.readline().decode().strip()
        return ack

    def close(self):
        try: self.ser.flush()
        except: pass

        try: self.ser.reset_input_buffer()
        except: pass

        try: self.ser.reset_output_buffer()
        except: pass

        try: self.ser.dtr = False
        except: pass

        try: self.ser.close()
        except: pass

        del self.ser
        time.sleep(0.5)  # let kernel release device
