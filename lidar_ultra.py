#!/usr/bin/env python3
"""
standalone_lidar.py
A stand-alone, No-ROS script for Raspberry Pi to create a 2D LIDAR
using a servo and an ultrasonic sensor.
- It controls the hardware directly.
- It performs the 150-degree sweep.
- It publishes the final 151-point scan to an MQTT topic.
"""
import paho.mqtt.client as mqtt
import time
import json
from gpiozero import Servo, DistanceSensor
from gpiozero.pins.pigpio import PiGPIOFactory

# --- Hardware Pin Setup (!! YOU MUST CHANGE THESE !!) ---
SERVO_PIN = 25      # BCM pin number for servo signal
ULTRA_TRIG_PIN = 23 # BCM pin number for ultrasonic trigger
ULTRA_ECHO_PIN = 24 # BCM pin number for ultrasonic echo

# --- MQTT Setup ---
MQTT_BROKER_HOST = "localhost" # Running on the same Pi
MQTT_TOPIC_LIDAR = "robot/lidar_scan"

# --- Global Hardware Objects ---
factory = PiGPIOFactory()
my_servo = Servo(
    SERVO_PIN,
    pin_factory=factory,
    min_pulse_width=0.5/1000,
    max_pulse_width=2.5/1000
)
sensor = DistanceSensor(
    echo=ULTRA_ECHO_PIN,
    trigger=ULTRA_TRIG_PIN,
    max_distance=4,
    pin_factory=factory
)

def map_angle_to_servo(angle_deg):
    """Converts a 0-180 degree angle to the servo's -1 to 1 range."""
    # Clamps the angle to the valid 0-180 range
    if angle_deg < 0: angle_deg = 0
    if angle_deg > 180: angle_deg = 180
    return ((angle_deg / 180.0) * 2.0) - 1.0

def get_clamped_distance(distance_m):
    """Clamps distance to valid range (0.01m to 4.0m)"""
    if distance_m > 4.0:
        return 4.0
    elif distance_m <= 0.01:
        return 0.01
    return round(distance_m, 3)

def main_loop():
    """Runs the main sweep and publishing loop."""
    
    # --- Setup MQTT Client ---
    client = mqtt.Client("LidarScanner")
    try:
        client.connect(MQTT_BROKER_HOST)
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        print("Make sure the 'mosquitto' service is running.")
        return
    client.loop_start() # Handles reconnects automatically
    print("Connected to MQTT Broker.")

    # --- CHANGED: Now a 151-point list (0-150 degrees) ---
    scan_data = [0.0] * 151 # Pre-allocate list
    scan_time_per_step = 0.08 # 80ms.
    
    print("Starting Lidar Scans (0-150 degrees)...")
    while True:
        # 1. Perform the 150-degree sweep
        print("Starting new sweep...")
        
        # --- CHANGED: Loop from 0 to 150 (inclusive) ---
        for i in range(151):
            # 1a. Move the servo
            servo_value = map_angle_to_servo(i)
            my_servo.value = servo_value
            
            # 1b. Wait for servo to move
            time.sleep(scan_time_per_step)
            
            # 1c. Take a measurement
            distance = get_clamped_distance(sensor.distance)
            scan_data[i] = distance

        # 2. Sweep complete, publish the full scan
        print(f"Sweep complete. Publishing {len(scan_data)} points.")
        
        # We package the data as a JSON string to send it
        payload = json.dumps(scan_data)
        
        # Publish to the MQTT topic
        client.publish(MQTT_TOPIC_LIDAR, payload)
        
        # 3. Reset servo to 0-degree position
        my_servo.value = map_angle_to_servo(0)
        
        # 4. Wait a bit before the next scan
        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main_loop()
    except KeyboardInterrupt:
        print("\nShutting down scanner.")
    finally:
        # Clean up GPIO on exit
        print("Cleaning up GPIO...")
        if my_servo:
            my_servo.value = map_angle_to_servo(75) # Center servo in new range
            time.sleep(0.5)
            my_servo.close()
        if sensor:
            sensor.close()
        print("GPIO cleanup complete.")
