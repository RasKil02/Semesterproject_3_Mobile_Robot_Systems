import time
from machine import Pin, PWM, UART

def supplyMotor(supplyID: int, duty: int = int(0.4 * 1023), duration: float = 2.0):
    """
    supplyID: GPIO pin number used for PWM
    duty: PWM duty cycle (0–1023 on ESP8266 / 0–65535 on RP2040)
    duration: how long the motor runs (seconds)
    """
    if supplyID < 0 or supplyID > 3:
        print('Invalid supply number')
        return

    pin = supplyID+15
    
    # Initialize PWM on the selected pin
    pwm = PWM(Pin(pin))
    pwm.freq(1000)  # 1 kHz typical PWM frequency

    print("Activating supply motor", supplyID)
    pwm.duty(duty)  # Start motor at desired duty-cycle

    time.sleep(duration)

    # Turn motor off
    pwm.duty(0)
    pwm.deinit()  # Stop PWM to free hardware resources
    print("Deactivating supply motor", supplyID)

def main():
    #sæt pin 21 high
    # Vælg en pin, f.eks. GPIO 15
    pin21 = Pin(21, Pin.OUT)

    # Sæt pin high (3.3V)
    pin21.value(1)
    uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

    while True:
        if uart.any():
            data = uart.read(1)  # læs én byte
            if data:
                supply_id_str = data.decode().strip()  # decode til streng, fjern evt. whitespace
                print("Received:", supply_id_str)
                try:
                    supply_id = int(supply_id_str)  # konverter til int
                    supplyMotor(supply_id)          # kør motor med supply_id
                except ValueError:
                    print("Invalid data received:", supply_id_str)




if __name__ == "__main__":
    main()