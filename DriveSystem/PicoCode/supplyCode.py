import time
from machine import Pin, PWM, UART

def supplyMotor(supplyID: int, duty: int = int(0.4 * 1023), duration: float = 2.0):
    """
    supplyID: GPIO pin number used for PWM
    duty: PWM duty cycle (0–1023 on ESP8266 / 0–65535 on RP2040)
    duration: how long the motor runs (seconds)
    """

    # Initialize PWM on the selected pin
    pwm = PWM(Pin(supplyID))
    pwm.freq(1000)  # 1 kHz typical PWM frequency

    print("Activating supply motor", supplyID)
    pwm.duty(duty)  # Start motor at desired duty-cycle

    time.sleep(duration)

    # Turn motor off
    pwm.duty(0)
    pwm.deinit()  # Stop PWM to free hardware resources
    print("Deactivating supply motor", supplyID)

def main():
    uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

    while True:
        if uart.any():
            data = uart.read().decode()
            print("Received:", data)
        supplyMotor(data)


if __name__ == "__main__":
    main()