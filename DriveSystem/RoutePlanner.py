import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class RoutePlanner(Node):
    def __init__(self):
        super().__init__('route_planner')
        self.declare_parameter('rate_hz', 20.0)
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.rate_hz = max(1.0, min(self.rate_hz, 100.0))  # bounds like before
        self.dt = 1.0 / self.rate_hz

    def _make_msg(self, v: float = 0.0, w: float = 0.0) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        return msg

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
        # out
        self.driveStraight(speed, duration)
        # rotate 90° in ~2s
        self.rotate(angular_speed, 2.0)
        # drop
        time.sleep(2.0)
        self.dropSupply(supplies)
        # return with same |speed| and same duration
        self.ReturnHome(speed, duration)

    def DegreesToAngularSpeed(self, degrees: float) -> float:
        radians = degrees * (math.pi / 180.0)
        # choose angular speed so we complete 'degrees' in 2 seconds
        return radians / 2.0

    def ReturnHome(self, speed: float, duration: float):
        self.get_logger().info('Returning Home')
        self.rotate(self.DegreesToAngularSpeed(-90), 2.0)   # rotate back ~90°
        self.driveStraight(-abs(float(speed)), float(duration))  # same duration back

    def chooseRoute(self, command: str):
        dest = self.destDecision(command)
        supplies = self.supplyDecision(command)
        rotation_speed = self.DegreesToAngularSpeed(90)

        speed = 0.08  # m/s
        durations = {1: 2.0, 2: 4.0, 3: 6.0, 4: 8.0}

        if dest in durations:
            self.get_logger().info(f'Routing to Location {dest}')
            self.executeRoute(supplies, speed, durations[dest], rotation_speed)
        else:
            self.get_logger().warn('Unknown Destination')