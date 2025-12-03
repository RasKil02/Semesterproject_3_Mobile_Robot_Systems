import time
import sys
from machine import Pin, PWM

def supplyMotor(supplyID: int, duty: int = int(0.4 * 65535), duration: float = 2.0):
    """
    supplyID: motor number (0–3)
    duty: PWM duty cycle (0–65535 on RP2040)
    duration: how long the motor runs (seconds)
    """

    if supplyID < 0 or supplyID > 3:
        print('Invalid supply number')
        return

    pin = supplyID + 15  # Pins 15,16,17,18 for motors

    pwm = PWM(Pin(pin))
    pwm.freq(1000)

    print("Activating supply motor", supplyID)
    pwm.duty_u16(duty)

    time.sleep(duration)

    pwm.duty_u16(0)
    pwm.deinit()
    print("Deactivating supply motor", supplyID)


def send_ack(supplyID):
    """Send confirmation back to the Pi."""
    msg = f"received {supplyID}\n"
    sys.stdout.write(msg)
    sys.stdout.flush()   # ensure it is sent immediately


def main():
    # LED for debugging
    pin22 = Pin(22, Pin.OUT)
    pin22.value(1)

    print("Pico ready. Listening on USB serial...")

    while True:
        data = sys.stdin.buffer.read(1)   # read ONE byte from USB
        if data:
            try:
                supply_id_str = data.decode().strip()
                print("Received:", supply_id_str)

                supply_id = int(supply_id_str)

                # Run the motor
                supplyMotor(supply_id)

                # Send confirmation back to the Pi
                send_ack(supply_id)

            except Exception as e:
                print("Error:", e)


main()
