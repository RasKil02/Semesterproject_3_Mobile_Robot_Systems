import time
import sys
from machine import Pin, PWM

def run_motor(pin_num, duty, duration):
    pin = Pin(pin_num, Pin.OUT)
    pwm = PWM(pin)
    pwm.freq(5000)
    pwm.duty_u16(duty)

    print(f"[PICO] motor on pin {pin_num}, {duration}s")
    sys.stdout.write("\n")

    end = time.ticks_ms() + int(duration * 1000)

    # keep USB alive
    while time.ticks_ms() < end:
        time.sleep(0.01)

    pwm.duty_u16(0)
    pwm.deinit()
    pin.value(0)

    print(f"[PICO] motor on pin {pin_num} stopped")
    sys.stdout.write("\n")

def send_ack(supplyID):
    sys.stdout.write(f"received {supplyID}\n")

def main():
    print("[PICO] ready")
    sys.stdout.write("\n")

    while True:
        data = sys.stdin.read(1)   # NON-BLOCKING on USB CDC

        if not data:
            time.sleep(0.01)
            continue
        
        try:
            supply_id = int(data)
        except:
            print("[PICO] bad input")
            continue

        # only motor 1 and 2
        if supply_id == 1:
            run_motor(17, int(0.7 * 65535), 1.0)

        elif supply_id == 2:
            run_motor(18, int(0.7 * 65535), 1.0)

        else:
            print("[PICO] id out of range")
            continue

        send_ack(supply_id)

main()

