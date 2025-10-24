DifferentialDrive class is used for basic driving functionallity. 

RoutePlanner is used for descision making based on protocol read from microphone

___________________________
Test code how does it work?

MoveTest (din node) = publisher på /cmd_vel (dette er et topic)

turtlebot3_node (bringup) = subscriber på /cmd_vel (altså turtlebot subscriber på det topic hvor vi ligger koden op)

main.py = starter ROS (rclpy.init()), opretter MoveTest, og lader den køre (så publish sker i din løkke/timer).