"""
Windows MQTT Publisher for TurtleBot3

- Sends movement durations to TurtleBot3 over MQTT
- Designed to work with the rclpy subscriber on the TurtleBot
"""

import paho.mqtt.client as mqtt

# ---------------- Adjustable Parameters ----------------
TURTLEBOT_IP = "192.168.64.89"
MQTT_PORT = 1883
MQTT_TOPIC = "turtlebot/move"
# -------------------------------------------------------

def send_move_command(duration):
    """Send a movement duration (in seconds) to the TurtleBot."""
    try:
        client = mqtt.Client()
        client.connect(TURTLEBOT_IP, MQTT_PORT, keepalive=60)
        client.publish(MQTT_TOPIC, str(duration))
        client.disconnect()
        print(f"[INFO] Sent move command: {duration} seconds")
    except Exception as e:
        print(f"[ERROR] Could not send MQTT message: {e}")

def main():
    print("TurtleBot3 MQTT Controller")
    while True:
        try:
            duration = float(input("Enter movement duration in seconds (or 0 to quit): "))
            if duration <= 0:
                print("Exiting...")
                break
            send_move_command(duration)
        except ValueError:
            print("[WARNING] Please enter a valid number.")

if __name__ == "__main__":
    main()
