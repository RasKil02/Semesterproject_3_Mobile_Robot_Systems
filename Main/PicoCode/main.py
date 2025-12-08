import sys
import time
import select
from machine import Pin, PWM

# ---------- MOTOR FUNCTION ----------
def run_motor(pin_num, duty, duration):
    pin = Pin(pin_num, Pin.OUT)
    pwm = PWM(pin)
    pwm.freq(5000)
    pwm.duty_u16(duty)

    print(f"[PICO] motor {pin_num} ON for {duration}s")
    sys.stdout.flush()

    end = time.ticks_ms() + int(duration * 1000)

    # Keep USB alive + timeout-safe
    while time.ticks_ms() < end:
        time.sleep(0.01)

    # --- ALWAYS shut motor off, even if USB freezes ---
    pwm.duty_u16(0)
    pwm.deinit()
    pin.value(0)

    print(f"[PICO] motor {pin_num} OFF")
    sys.stdout.flush()


def send_ack(supply_id):
    sys.stdout.write(f"ACK {supply_id}\n")
    sys.stdout.flush()


# ---------- NON-BLOCKING SERIAL SETUP ----------
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)


print("[PICO] READY\n")
sys.stdout.flush()

# ---------- MAIN LOOP ----------
while True:

    # Check if any input available (non-blocking)
    if poller.poll(0):

        line = sys.stdin.readline().strip()

        # Ignore empty lines or garbage
        if not line.isdigit():
            print("[PICO] invalid input")
            sys.stdout.flush()
            continue

        supply = int(line)

        # ------------ COMMANDS -------------
        if supply == 1:
            run_motor(17, int(0.7 * 65535), 1.0)

        elif supply == 2:
            run_motor(18, int(0.7 * 65535), 1.0)

        else:
            print("[PICO] ID out of range")
            sys.stdout.flush()
            continue

        send_ack(supply)

    time.sleep(0.01)
