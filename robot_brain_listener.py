#!/usr/bin/env python3
"""
robot_brain_listener.py
The "brain" of your robot.
- It connects to the MQTT broker.
- It subscribes to the 'robot/lidar_scan' topic.
- It receives the scan data and can make decisions.
"""
import paho.mqtt.client as mqtt
import json

MQTT_BROKER_HOST = "localhost"
MQTT_TOPIC_LIDAR = "robot/lidar_scan"

def on_connect(client, userdata, flags, rc):
    """Callback for when we connect to the broker."""
    print(f"Connected with result code {rc}")
    
    # Subscribe to the LIDAR topic as soon as we connect
    client.subscribe(MQTT_TOPIC_LIDAR)
    print(f"Subscribed to {MQTT_TOPIC_LIDAR}")

def on_message(client, userdata, msg):
    """Callback for when a message is received."""
    
    # Check if the message is from our LIDAR topic
    if msg.topic == MQTT_TOPIC_LIDAR:
        print("Received LIDAR scan!")
        
        try:
            # 1. Decode the message (it's a JSON string)
            # The .decode('utf-8') turns the raw bytes into a string
            scan_data = json.loads(msg.payload.decode('utf-8'))
            
            # 2. Now you have the 180-point list of distances
            # print(f"Scan has {len(scan_data)} points.")
            
            # --- YOUR NAVIGATION LOGIC GOES HERE ---
            # Check for obstacles every 30 degrees
            
            print("--- 30-Degree Scan Report ---")
            obstacle_detected = False
            obstacle_threshold = 0.3 # 30cm
            
            # We will check 0, 30, 60, 90, 120, 150, and 179 (for 180)
            for angle in range(0, 181, 30):
                # Ensure we don't go out of bounds (index 180 doesn't exist)
                index = min(angle, 179) 
                
                distance = scan_data[index]
                # Format to 2 decimal places for cleaner output
                print(f"  Angle {angle}°: {distance:.2f} meters") 
                
                if distance < obstacle_threshold:
                    obstacle_detected = True
            
            print("-----------------------------")
            
            # Make a single decision based on the scan
            if obstacle_detected:
                print("Obstacle detected within scan range! Stopping motors.")
                # When you have motors, you would publish a command:
                # client.publish("robot/motors", "STOP")
            else:
                print("Path is clear. Moving forward.")
                # client.publish("robot/motors", "FORWARD")

        except json.JSONDecodeError:
            print("Error: Could not decode LIDAR data.")
        except IndexError:
            print("Error: LIDAR data list was not the right size.")

def main():
    # This is the v1.x compatible way to create a client
    client = mqtt.Client("RobotBrain")
    
    # Assign the callback functions
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER_HOST)
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        print("Make sure the 'mosquitto' service is running.")
        return

    # loop_forever() is a blocking call that handles all traffic
    # and automatically reconnects if it gets disconnected.
    print("Robot Brain is listening...")
    client.loop_forever()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nShutting down Robot Brain.")

