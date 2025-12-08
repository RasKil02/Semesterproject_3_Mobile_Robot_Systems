import serial
import time

class PicoMotorController:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # allow Pico to finish USB reset

        # Flush leftover garbage
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send_supply_id(self, supply_id):
        msg = f"{supply_id}\n".encode()
        self.ser.write(msg)
        self.ser.flush()

        # wait for ack
        ack = self.ser.readline().decode().strip()
        return ack

    def close(self):
        self.ser.close()


def main():
    pico = PicoMotorController(
        "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6613008e37c6f34-if00"
    )

    print("Sending 2...")
    print("Pico:", pico.send_supply_id(2))

    time.sleep(3)

    print("Sending 1...")
    print("Pico:", pico.send_supply_id(1))

    pico.close()


if __name__ == "__main__":
    main()
