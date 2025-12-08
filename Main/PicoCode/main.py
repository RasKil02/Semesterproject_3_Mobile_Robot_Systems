import time
import sys
from machine import Pin, PWM


def run_motor(duration):
    end = time.ticks_add(time.ticks_ms(), int(duration*1000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        time.sleep(0.01)  # keep USB alive


def supplyMotor(supplyID: int, duty: int = int(0.5 * 65535), duration: float = 10.0):
    """
    supplyID: motor number (0–3)
    duty: PWM duty cycle (0–65535 on RP2040)
    duration: how long the motor runs (seconds)
    """

    if supplyID < 0 or supplyID > 3:
        print('Invalid supply number')
        return

    pin_num = supplyID + 15  # Pins 15,16,17,18 for motors
    pin_obj = Pin(pin_num, Pin.OUT)

    # Start PWM
    pwm = PWM(pin_obj)
    pwm.freq(5000)

    print("Activating supply motor", supplyID)
    pwm.duty_u16(duty)

    run_motor(duration)

    # Turn off PWM
    pwm.duty_u16(0)
    pwm.deinit()
    pin_obj.value(0)

    print("Deactivating supply motor", supplyID)


def send_ack(supplyID):
    """Send confirmation back to the Pi."""
    msg = f"received {supplyID}\n"
    sys.stdout.write(msg)
    sys.stdout.flush()


def main():
    # Debug LED on pin 22
    pin20 = Pin(20, Pin.OUT)
    pin20.value(1)

    print("Pico ready. Listening on USB serial...")

    while True:
        data = sys.stdin.buffer.read(1)   # read ONE byte from USB
        if data:
            try:
                supply_id_str = data.decode().strip()
                print("Received:", supply_id_str)

                supply_id = int(supply_id_str)

                supplyMotor(supply_id)

                send_ack(supply_id)

            except Exception as e:
                print("Error:", e)


main()
