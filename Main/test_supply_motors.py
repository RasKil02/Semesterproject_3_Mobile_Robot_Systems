from DriveSystem.PicoMotorController import PicoMotorController

picosender = PicoMotorController()  

def main():
    print("Sending supply ID 2 to Pico...")
    picosender.send_supply_id(2)

if __name__ == "__main__":
    main()
    picosender.close()