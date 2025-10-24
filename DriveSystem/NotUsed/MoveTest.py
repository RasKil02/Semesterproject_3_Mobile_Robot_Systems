<<<<<<< HEAD:DriveSystem/MoveTest.py
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
=======
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # Must have package, defines message type
>>>>>>> 356d484 (Created RoutePlanner class, and updated main):DriveSystem/NotUsed/MoveTest.py

# Test node to move robot forward, turn, and backward
class MoveTest(Node):
    def __init__(self):
<<<<<<< HEAD:DriveSystem/MoveTest.py
        super().__init__('move_test')
        self.declare_parameter('v_forward', 0.06)
        self.declare_parameter('v_backward', -0.06)
        self.declare_parameter('t_forward', 2.0)
        self.declare_parameter('t_backward', 2.0)
        self.declare_parameter('turn_deg', 90.0)
        self.declare_parameter('w_turn', 0.6)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
=======
        super().__init__('move_test') # initialize node with name 'move_test' name, super() is important

        # Parametre (kan ændres fra CLI (when initializing node in terminal))
        self.declare_parameter('v_forward', 0.06)
        self.declare_parameter('v_backward', -0.06)   # m/s (negativ)
        self.declare_parameter('t_forward', 2.0) 
        self.declare_parameter('t_backward', 2.0) 

        self.declare_parameter('turn_deg', 90.0) 
        self.declare_parameter('w_turn', 0.6)         # rad/s (positiv = venstre)

        # VERY IMPORTANT line, creates publisher, which will send Twist messages to /cmd_vel topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Rate to send commands at (Hz), is used later to control loop timing
>>>>>>> 356d484 (Created RoutePlanner class, and updated main):DriveSystem/NotUsed/MoveTest.py
        self.rate_hz = 20.0

    # Function to send velocity commands for a certain duration
    def send_cmd(self, vx: float, wz: float, duration: float):
<<<<<<< HEAD:DriveSystem/MoveTest.py
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
=======
        """Publisér vx/wz i 'duration' sekunder med fast rate."""

        # Create Twist message with desired velocities
        twist = Twist() 
        twist.linear.x = float(vx)
        twist.angular.z = float(wz)

        # Calculate end time and period between messages
        t_end = time.time() + max(0.0, float(duration))
        period = 1.0 / self.rate_hz
>>>>>>> 356d484 (Created RoutePlanner class, and updated main):DriveSystem/NotUsed/MoveTest.py

        # Loop to publish commands at fixed rate until duration is reached and while ROS is running
        while rclpy.ok() and time.time() < t_end:
            self.pub.publish(twist) # Sends velocity command
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period) # pause to maintain rate

    # Function to stop the robot, does so by sending an empty Twist message
    def stop(self, secs: float = 0.2):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # twist zeroes by default
        self.pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(max(0.0, secs))

    # Main sequence to move forward, turn, and move backward
    def run_sequence(self):
<<<<<<< HEAD:DriveSystem/MoveTest.py
=======

        # Hent parametre
>>>>>>> 356d484 (Created RoutePlanner class, and updated main):DriveSystem/NotUsed/MoveTest.py
        v_f = float(self.get_parameter('v_forward').value)
        v_b = float(self.get_parameter('v_backward').value)
        t_f = float(self.get_parameter('t_forward').value)
        t_b = float(self.get_parameter('t_backward').value)
        turn_deg = float(self.get_parameter('turn_deg').value)
        w_turn = float(self.get_parameter('w_turn').value)
        t_turn = abs(math.radians(turn_deg) / w_turn) if w_turn != 0 else 0.0
        wz = math.copysign(abs(w_turn), turn_deg)

        # Output info about sequence
        self.get_logger().info('Sekvens: frem -> drej -> tilbage')
        self.send_cmd(v_f, 0.0, t_f);     self.stop()
        self.send_cmd(0.0, wz, t_turn);   self.stop()
        self.send_cmd(v_b, 0.0, t_b);     self.stop(0.3)

