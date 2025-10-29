import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('rate_hz', 20.0)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.rate_hz = max(1.0, min(self.rate_hz, 100.0))  # bounds
        self.dt = 1.0 / self.rate_hz

    def _make_msg(self, v: float = 0.0, w: float = 0.0) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        return msg

    def publish_vw_for_duration(self, v: float, w: float, duration: float):
        t_end = time.time() + max(0.0, float(duration))
        while rclpy.ok() and time.time() < t_end:
            self.pub.publish(self._make_msg(v, w))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.dt)

    def stop(self, pause: float = 0.2):
        self.pub.publish(self._make_msg(0.0, 0.0))
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, pause))

    def destDecision(self, command: str) -> int:
        table = {"00000000": 1, "00000001": 2, "00000010": 3, "00000011": 4}
        return table.get(command[:8], 0)

    def supplyDecision(self, command: str) -> int:
        table = {"00000000": 0, "00000001": 1, "00000010": 2, "00000011": 3}
        return table.get(command[8:16], 0)

    def driveStraight(self, speed: float, duration: float):
        self.publish_vw_for_duration(speed, 0.0, duration)
        self.stop(0.3)

    def rotate(self, angular_speed: float, duration: float):
        self.publish_vw_for_duration(0.0, angular_speed, duration)
        self.stop(0.3)
    
    def dropSupply(self, supplies: int):
        if supplies <= 0:
            self.get_logger().info('No supplies to drop')
            return
        for i in range(1, supplies + 1):
            self.get_logger().info(f'Dropping {i} supply' if i == 1 else f'Dropping {i} supplies')

    def executeRoute(self, supplies: int, speed: float, duration: float, angular_speed: float):
        self.driveStraight(speed, duration)
        self.rotate(angular_speed, 2.0)
        self.dropSupply(supplies)
        self.ReturnHome(duration)

    def DegreesToAngularSpeed(self, degrees: float) -> float:
        radians = degrees * (3.14159265 / 180.0)
        angular_speed = radians / 2.0  # Assuming we want to complete the rotation in 1 second
        return angular_speed
    
    def ReturnHome(self, duration: float):
        self.get_logger().info('Returning Home')
        self.rotate(self.DegreesToAngularSpeed(-90), 2.0)  # Rotate -90 degrees
        self.driveStraight(-0.15, duration)  # Drive straight for 8 seconds at 0.15 m/s

    def chooseRoute(self, command: str):
        dest = self.destDecision(command)
        supplies = self.supplyDecision(command)
        rotation = self.DegreesToAngularSpeed(90)

        if dest == 1:
            self.executeRoute(supplies, 0.8, 2, rotation) # 0.22 is max speed
            self.get_logger().info('Routing to Location 1')
        elif dest == 2:
            self.executeRoute(supplies, 0.8, 4, rotation)
            self.get_logger().info('Routing to Location 2')
        elif dest == 3:
            self.executeRoute(supplies, 0.8, 6, rotation)
            self.get_logger().info('Routing to Location 3')
        elif dest == 4:
            self.executeRoute(supplies, 0.8, 8, rotation)
            self.get_logger().info('Routing to Location 4')
        else:
            self.get_logger().info('Unknown Destination')
