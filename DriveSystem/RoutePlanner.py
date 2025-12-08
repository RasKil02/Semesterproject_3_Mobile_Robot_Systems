import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from DriveSystem.PicoMotorController import PicoMotorController
picosender = PicoMotorController(
        "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6613008e37c6f34-if00"
    )
#from machine import Pin
import serial
import time


class RoutePlanner(Node): # gør at klassen arber fra node klassen, så vi kan bruge rclpy funktioner
    def __init__(self):
        super().__init__('route_planner') # initialize the Node class with the name 'route_planner'

        # Fixed internal settings
        self.rate_hz = 40.0                     # <- Hard-coded rate
        self.rate_hz = max(1.0, min(self.rate_hz, 100.0))  # clamp just in case
        self.dt = 1.0 / self.rate_hz

        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10) # creates a topic named cmd_vel of type TwistStamped(sends velocity commands to the robot)
        self.get_logger().info(f"RoutePlanner initialized with fixed rate: {self.rate_hz} Hz")

         # ÅBN UART ÉN GANG
        #self.ser = serial.Serial(
         #   port="/dev/serial0",
          #  baudrate=9600,
           # timeout=1)

    # Makes a TwistStamped message with given linear and angular velocities
    def _make_msg(self, v: float = 0.0, w: float = 0.0) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        return msg

    # Publishes velocity commands for a specified duration
    def publish_vw_for_duration(self, v: float, w: float, duration: float):
        duration = max(0.0, float(duration))
        end_t = time.monotonic() + duration
        next_tick = time.monotonic()
        while rclpy.ok() and time.monotonic() < end_t:
            self.pub.publish(self._make_msg(v, w))
            rclpy.spin_once(self, timeout_sec=0.0)
            next_tick += self.dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
        # ensure stop after each segment
        self.pub.publish(self._make_msg(0.0, 0.0))

    # Stops the robot with an optional pause
    def stop(self, pause: float = 0.2):
        self.pub.publish(self._make_msg(0.0, 0.0))
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, pause))

    # Reads first 6 bits to determine destination
    def destDecision(self, command: str) -> int:
        # first 6 bits determine destination
        print("Command received in RoutePlanner:", command)
        table = {"000000": 0, "000001": 1, "000010": 2, "000011": 3}
        dest_bits = command[:6]
        return table.get(dest_bits, 0)
    
    # Reads next 6 bits to determine number of supplies
    def supplyDecision(self, command: str) -> int:
        # next 6 bits determine number of supplies
        table = {"000000": 0, "000001": 1, "000010": 2, "000011": 3}
        supply_bits = command[6:12]
        return table.get(supply_bits, 0)

    # Drives straight for a given speed and duration
    def driveStraight(self, speed: float, duration: float):
        print("In driveStraight with speed:", speed, "duration:", duration)
        self.publish_vw_for_duration(speed, 0.0, duration)
        self.stop(0.3)

    # Rotates the robot at a given angular speed for a specified duration
    def rotate(self, angular_speed: float, duration: float):
        self.publish_vw_for_duration(0.0, angular_speed, duration)
        self.stop(0.3)

    def dropSupply(self, supplies: int):
        if supplies < 0 or supplies > 3:
            self.get_logger().info('Invalid supply number')
            return

        print("Dropping supply:", supplies)
        picosender.send_supply_id(supplies)
      


    # Executes the full route: drive out, rotate, drop supplies, return home
    def executeRoute(self, supplies: int, speed: float, duration: float, angular_speed: float):
        # out
        print("In executeRoute with supplies:", supplies, "speed:", speed, "duration:", duration, "angular_speed:", angular_speed)
        self.driveStraight(speed, duration)
        # rotate 90° in ~2s
        self.rotate(angular_speed, 2.0)
        # drop
        time.sleep(2.0)
        # her skal der skrives noget kode i forhold til at styre drop mekanismen
        self.dropSupply(supplies)
        time.sleep(2.0)

        # return with same |speed| and same duration
        self.ReturnHome(speed, duration)

    # Converts degrees to angular speed (radians/sec)
    def DegreesToAngularSpeed(self, degrees: float) -> float:
        radians = degrees * (math.pi / 180.0)
        # choose angular speed so we complete 'degrees' in 2 seconds
        return radians / 2.0

    # Returns home by rotating back and driving straight
    def ReturnHome(self, speed: float, duration: float):
        print("In return home speed:", speed, "duration:", duration)
        self.get_logger().info('Returning Home')
        self.rotate(self.DegreesToAngularSpeed(-90), 2.0)   # rotate back ~90°
        self.driveStraight(-abs(float(speed)), float(duration))  # same duration back

    # Chooses route based on command and executes it. command is a 12 bit string
    def chooseRoute(self, command: str):
        dest = self.destDecision(command)
        print("In chooseRoute, destination is:", dest)
        supplies = self.supplyDecision(command)
        rotation_speed = self.DegreesToAngularSpeed(90)

        speed = 0.08  # m/s
        durations = {0: 1.0, 1: 3.0, 2: 5.0, 3: 7.0}

        if dest in durations:
            self.get_logger().info(f'Routing to Location {dest}')
            self.executeRoute(supplies, speed, durations[dest], rotation_speed)
        else:
            self.get_logger().warn('Unknown Destination')