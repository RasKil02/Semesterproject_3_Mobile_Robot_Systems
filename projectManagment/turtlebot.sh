IP="172.20.10.3"  # Skift til din rigtige IP

echo "Forbinder til TurtleBot på $IP..."
echo "Brugernavn: pi"          # ← RETTET HER! Skal være "pi"
echo "Password: turtlebot"     # ← Password er korrekt
echo ""

# Automatisk login – sshpass sender password INDEN ssh tager terminalen
sshpass -p "turtlebot" ssh pi@$IP "~/start_turtlebot.sh"

# Fil der skal laves på robotten.
nano ~/start_turtlebot.sh

#!/bin/bash
# Session A - echo cmd_vel
screen -dmS A bash -c "source /opt/ros/jazzy/setup.bash; ros2 topic echo /cmd_vel geometry_msgs/TwistStamped; exec bash"

# Session B - motor power + python program
screen -dmS B bash -c "source /opt/ros/jazzy/setup.bash; ros2 service call /motor_power std_srvs/srv/SetBool '{data: true}'; cd ~/code_ws/src/Semesterproject_3_Mobile_Robot_Systems; source venv/bin/activate; PYTHONPATH=\"\$PWD:\$PYTHONPATH\" python3 -m Main.main; exec bash"

# Session C - robot launch
screen -dmS C bash -c "source /opt/ros/jazzy/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_bringup robot.launch.py; exec bash"

echo "Screens A, B og C er startet med de ønskede kommandoer!"
