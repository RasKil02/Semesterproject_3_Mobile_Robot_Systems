# move_test.py
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test')

        # Parametre (kan ændres fra CLI)
        self.declare_parameter('v_forward', 0.06)     # m/s
        self.declare_parameter('v_backward', -0.06)   # m/s (negativ)
        self.declare_parameter('t_forward', 2.0)      # sek
        self.declare_parameter('t_backward', 2.0)     # sek

        self.declare_parameter('turn_deg', 90.0)      # grader
        self.declare_parameter('w_turn', 0.6)         # rad/s (positiv = venstre)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate_hz = 20.0

    def send_cmd(self, vx: float, wz: float, duration: float):
        """Publisér vx/wz i 'duration' sekunder med fast rate."""
        twist = Twist()
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)

        t_end = time.time() + max(0.0, float(duration))
        period = 1.0 / self.rate_hz
        while rclpy.ok() and time.time() < t_end:
            self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def stop(self, secs: float = 0.2):
        """Stop blødt og hold stille et øjeblik."""
        self.pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, secs))

    def run_sequence(self):
        # Hent parametre
        v_f = float(self.get_parameter('v_forward').value)
        v_b = float(self.get_parameter('v_backward').value)
        t_f = float(self.get_parameter('t_forward').value)
        t_b = float(self.get_parameter('t_backward').value)

        turn_deg = float(self.get_parameter('turn_deg').value)
        w_turn = float(self.get_parameter('w_turn').value)

        # Beregn varighed for drej baseret på ønsket vinkel og vinkelhastighed
        turn_rad = math.radians(turn_deg)
        t_turn = abs(turn_rad / w_turn) if w_turn != 0 else 0.0
        wz = math.copysign(abs(w_turn), turn_rad)  # drej i retning af vinklen

        self.get_logger().info('Sekvens: frem -> drej -> tilbage')

        # Frem
        self.send_cmd(v_f, 0.0, t_f)
        self.stop()

        # Drej
        self.send_cmd(0.0, wz, t_turn)
        self.stop()

        # Tilbage
        self.send_cmd(v_b, 0.0, t_b)
        self.stop(0.3)
