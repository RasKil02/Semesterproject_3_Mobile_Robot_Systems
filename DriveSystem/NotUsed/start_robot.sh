#!/bin/bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Launch the robot nodes
ros2 launch /home/pi/turtlebot3_ws/install/turtlebot3_bringup/share/turtlebot3_bringup/launch/robot.launch.py &

# Give it a few seconds to initialize
sleep 5

# Start the MQTT subscriber script (adjust the path if different)
python3 /home/pi/mqtt_turtlebot_receiver.py
