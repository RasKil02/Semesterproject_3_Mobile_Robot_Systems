import sys
import time
from machine import Pin

# Setup pins as output
pin1 = Pin(17, Pin.OUT)
pin2 = Pin(18, Pin.OUT)

# Ensure both are low at start
pin1.low()
pin2.low()

def activate_pin(pin):
    pin.high()
    time.sleep(0.02)
    pin.low()

print("Waiting for USB serial input (1 or 2)...")

while True:
    char = sys.stdin.read(1)

    if not char:
        continue

    if char == "1":
        print("ACK:1")
        activate_pin(pin1)

    elif char == "2":
        print("ACK:2")
        activate_pin(pin2)