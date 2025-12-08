from DriveSystem.PicoMotorController import PicoMotorController
import time

def main():

    pico = PicoMotorController()
    
    print("Sending command 2...")
    ack = pico.send_supply_id(2)
    print("Pico replied:", ack)

    time.sleep(5)

    print("Sending command 1...")
    ack = pico.send_supply_id(1)
    print("Pico replied:", ack)

    pico.close()   # <---- Only close AFTER all commands
                    # DO NOT re-open in the same program

if __name__ == "__main__":
    main()
