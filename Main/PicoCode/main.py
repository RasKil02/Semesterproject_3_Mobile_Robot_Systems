import time
import sys
import select
from machine import Pin, PWM

# ----------------------------------------------------
# Safe flush for Thonny (FileIO has no .flush())
# ----------------------------------------------------
def safe_flush():
    try:
        sys.stdout.flush()
    except:
        pass


# ----------------------------------------------------
# Non-blocking input setup
# ----------------------------------------------------
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)


# ----------------------------------------------------
# Motor run function
# ----------------------------------------------------
def run_motor(pin_num, duty, duration):

    pin = Pin(pin_num, Pin.OUT)
    pwm = PWM(pin)
    pwm.freq(5000)
    pwm.duty_u16(duty)

    print(f"[PICO] Motor on pin {pin_num} running for {duration}s")
    safe_flush()

    end = time.ticks_add(time.ticks_ms(), int(duration * 1000))

    # Non-blocking loop to keep USB alive
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        time.sleep(0.01)

    pwm.duty_u16(0)
    pwm.deinit()
    pin.value(0)

    print(f"[PICO] Motor on pin {pin_num} stopped")
    safe_flush()
    
def send_ack(supplyID):
    """Send confirmation back to the Pi."""
    msg = f"received {supplyID}\n"
    sys.stdout.write(msg)
    sys.stdout.flush()

# ----------------------------------------------------
# Main loop
# ----------------------------------------------------
def main():

    # Debug pins (if needed)
    Pin(15, Pin.OUT).value(0)
    Pin(13, Pin.OUT).value(1)

    print("[PICO] Ready. Listening for commands...")
    safe_flush()

    while True:

        # Non-blocking USB serial read
        if poll.poll(0):
            line = sys.stdin.readline()

            if not line:
                continue

            try:
                supply_id = int(line.strip())
            except:
                print("[PICO] Invalid input")
                safe_flush()
                continue

            # Valid motor IDs: 1-2 → pins 17–18
            if 0 < supply_id < 3:
                pin_num = 16 + supply_id

                run_motor(
                    pin_num=pin_num,
                    duty=int(0.7 * 65535),
                    duration=1.0,
                )

                # ACK back to Raspberry Pi
                send_ack(supply_id)
                safe_flush()

            else:
                print("[PICO] ID out of range")
                safe_flush()

        time.sleep(0.01)


main()



