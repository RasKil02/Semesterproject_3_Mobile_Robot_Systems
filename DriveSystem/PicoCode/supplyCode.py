import time
from machine import Pin

def stateMachine()

def supplyMotor(self, supplyID: int):
        pin = Pin(supplyID, Pin.OUT):
        print('Activating supply motor' + str(supplyID))
        pin.value(1)  # Activate motor
        time.sleep(1)  # Run motor for 1 second
        pin.value(0)  # Deactivate motor
        print('Deactivating supply motor' + str(supplyID))

def main():
    supplyMotor()

if __name__ == "__main__":
    main()