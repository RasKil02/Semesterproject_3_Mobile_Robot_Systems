from DriveSystem.PicoMotorController import PicoMotorController
import time

picosender = PicoMotorController()  

def main():
    print("Sending supply ID 2 to Pico...")
    picosender.send_supply_id(2)

    time.sleep(4)

    print("Sending supply ID 1 to Pico...")
    picosender.send_supply_id(1)


if __name__ == "__main__":
    main()
    picosender.close()