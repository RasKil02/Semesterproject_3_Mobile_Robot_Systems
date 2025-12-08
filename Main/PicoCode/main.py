import sys
import time
from machine import Pin, PWM

# --- PWM setup ---
pwm_pin = PWM(Pin(15))   # Change to the GPIO pin you're using
pwm_pin.freq(1000)       # 1 kHz PWM
pwm_pin.duty_u16(0)      # Start OFF

def activate_pwm():
    pwm_pin.duty_u16(32768)   # 50% duty (0.5)
    time.sleep(2)             # ON for 2 seconds
    pwm_pin.duty_u16(0)       # OFF

print("Waiting for USB serial input (1 or 2)...")

while True:
    # Read one byte from USB CDC
    char = sys.stdin.read(1)

    if char is None:
        continue  # no data yet

    if char in ("1", "2"):
        print("Received:", char)
        activate_pwm()


