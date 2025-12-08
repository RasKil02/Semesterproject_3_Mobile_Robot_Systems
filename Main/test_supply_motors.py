from DriveSystem.PicoMotorController import PicoMotorController
import time

def main():

    pico = PicoMotorController()
    print("Sending command 2...")
    ack = pico.send_supply_id(2)
    print("Pico replied:", ack)
    pico.close()

    time.sleep(4)

    pico = PicoMotorController()
    print("Sending command 1...")
    ack = pico.send_supply_id(1)
    print("Pico replied:", ack)
    pico.close()


if __name__ == "__main__":
    main()
