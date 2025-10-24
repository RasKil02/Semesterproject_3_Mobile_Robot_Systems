import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test')
        self.declare_parameter('v_forward', 0.06)
        self.declare_parameter('v_backward', -0.06)
        self.declare_parameter('t_forward', 2.0)
        self.declare_parameter('t_backward', 2.0)
        self.declare_parameter('turn_deg', 90.0)
        self.declare_parameter('w_turn', 0.6)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.rate_hz = 20.0

    def send_cmd(self, vx: float, wz: float, duration: float):
        period = 1.0 / self.rate_hz
        t_end = time.time() + max(0.0, float(duration))
        while rclpy.ok() and time.time() < t_end:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(vx)
            msg.twist.angular.z = float(wz)
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop(self, secs: float = 0.2):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # twist zeroes by default
        self.pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, secs))

    def run_sequence(self):
        v_f = float(self.get_parameter('v_forward').value)
        v_b = float(self.get_parameter('v_backward').value)
        t_f = float(self.get_parameter('t_forward').value)
        t_b = float(self.get_parameter('t_backward').value)
        turn_deg = float(self.get_parameter('turn_deg').value)
        w_turn = float(self.get_parameter('w_turn').value)
        t_turn = abs(math.radians(turn_deg) / w_turn) if w_turn != 0 else 0.0
        wz = math.copysign(abs(w_turn), turn_deg)

        self.get_logger().info('Sekvens: frem -> drej -> tilbage')
        self.send_cmd(v_f, 0.0, t_f);     self.stop()
        self.send_cmd(0.0, wz, t_turn);   self.stop()
        self.send_cmd(v_b, 0.0, t_b);     self.stop(0.3)