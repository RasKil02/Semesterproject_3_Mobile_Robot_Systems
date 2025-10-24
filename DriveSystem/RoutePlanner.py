# Used for descision making based on protocol read from microphone

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate_hz = 20.0
        self.dt = 1.0 / self.rate_hz
    
    # Depends on how we use this we can make the robot go straight, turn, stop, etc.
    def publish_vw_for_duration(self, v: float, w: float, duration: float):
        t_end = time.time() + max(0.0, float(duration))
        while rclpy.ok() and time.time() < t_end:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.dt)

    def stop(self, pause: float = 0.2):
        self.pub.publish(Twist())  # zeros
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, pause))

    def destDecision(command: str) -> int:
        table = {"00000000": 1, "00000001": 2, "00000010": 3, "00000011": 4}
        return table.get(command[:8], 0)

    def supplyDescision(command: str) -> int:
        table = {"00000000": 0, "00000001": 1, "00000010": 2, "00000011": 3}
        return table.get(command[8:16], 0)

    def driveStraight(self, speed: float, duration: float):
        self.publish_vw_for_duration(self, speed, 0.0, duration) # Sets angular velocity to 0 for straight line
        self.stop(self, 0.3)

    def rotate(self, angular_speed: float, duration: float):
        self.publish_vw_for_duration(self, 0.0, angular_speed, duration) # Sets linear velocity to 0 for rotation
        self.stop(self, 0.3)

    def dropSupply(self, command: int):
        if self.supplyDescision(command) <= 0:
            self.get_logger().info('No supplies to drop')
            return
        for i in range(1, self.supplyDescision(command) + 1):
            if i == 1:
                self.get_logger().info('Dropping 1 supply')
            else:
                self.get_logger().info(f'Dropping {i} supplies')

    def executeRoute(self, supplies: int, speed: float, duration: float, angular_speed: float):
        self.driveStraight(self, speed, duration)
        self.rotate(self, angular_speed, 1)
        self.dropSupply(self, supplies)

    def chooseRoute(self, command: int, supplies: int):

        if self.destDecision(command) == 1:
            self.executeRoute(self, supplies, 0.1, 2, 0.5)
            self.get_logger().info('Routing to Location 1')

        elif self.destDecision(command) == 2:
            self.executeRoute(self, supplies, 0.1, 4, 0.5)
            self.get_logger().info('Routing to Location 2')

        elif self.destDecision(command) == 3:
            self.executeRoute(self, supplies, 0.1, 6, 0.5)
            self.get_logger().info('Routing to Location 3')

        elif self.destDecision(command) == 4:
            self.executeRoute(self, supplies, 0.1, 8, 0.5)
            self.get_logger().info('Routing to Location 4')

        else:
            self.get_logger().info('Unknown Destination')

        