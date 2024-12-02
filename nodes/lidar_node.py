import subprocess
import time
import json
import paho.mqtt.client as mqtt
import numpy as np
from threading import Thread

# ======================== Configuration ========================

# MQTT Settings
BROKER_ADDRESS = "localhost"  # Broker is running on the Pi itself
BROKER_PORT = 1883
TOPIC_LIDAR = "golfcart/lidar"

# Path to the `ldlidar_stl_node` executable
LDLIDAR_EXECUTABLE = "./ldlidar_stl_node"
LIDAR_MODEL = "LD19"
LIDAR_PORT = "/dev/ttyUSB1"  # Update with your LiDAR's USB port

# Initialize MQTT client
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}")

mqtt_client.on_connect = on_connect

try:
    mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
    exit(1)  # Exit if connection fails

mqtt_client.loop_start()

def publish_lidar_data(data):
    """Publish LiDAR data to MQTT."""
    try:
        message = json.dumps(data)
        mqtt_client.publish(TOPIC_LIDAR, message)
    except Exception as e:
        print(f"Failed to publish MQTT message: {e}")

def parse_sdk_output(line):
    """Parse a single line of SDK output."""
    try:
        if "angle:" not in line or "distance(mm):" not in line or "intensity:" not in line:
            return None  # Skip non-data lines

        # Extract the relevant parts
        parts = line.split(",")
        angle = float(parts[1].split(":")[1])
        distance_mm = float(parts[2].split(":")[1])
        intensity = int(parts[3].split(":")[1].strip("]\n"))  # Remove trailing brackets and newline
        distance_m = distance_mm / 1000.0  # Convert mm to meters
        return {"angle": angle, "distance": distance_m, "intensity": intensity}
    except Exception as e:
        print(f"Error parsing line: {line.strip()} - {e}")
        return None

def read_lidar_output():
    """Read output from the LiDAR SDK and publish to MQTT."""
    try:
        process = subprocess.Popen(
            [LDLIDAR_EXECUTABLE, LIDAR_MODEL, "serialcom", LIDAR_PORT],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        while True:
            line = process.stdout.readline()
            if not line:
                break

            print(f"Raw SDK Output: {line.strip()}")  # Log all SDK output

            if "[LDS][INFO]" in line:  # Filter for relevant data lines
                lidar_data = parse_sdk_output(line)
                if lidar_data:
                    # Convert to Cartesian and include in MQTT payload
                    angle, distance = lidar_data["angle"], lidar_data["distance"]
                    x, y = convert_to_cartesian(angle, distance)
                    mqtt_payload = {
                        "frame_id": "lidar_frame",
                        "timestamp": time.time(),
                        "data": {
                            "angle": angle,
                            "distance": distance,
                            "intensity": lidar_data["intensity"],
                            "x": x,
                            "y": y
                        }
                    }
                    publish_lidar_data(mqtt_payload)

    except Exception as e:
        print(f"Error reading LiDAR output: {e}")

def convert_to_cartesian(angle, distance):
    """Convert polar coordinates (angle, distance) to Cartesian (x, y) in meters."""
    angle_rad = np.deg2rad(angle)
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return x, y

def main():
    # Start reading LiDAR output
    thread = Thread(target=read_lidar_output, daemon=True)
    thread.start()

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        mqtt_client.disconnect()
        mqtt_client.loop_stop()

if __name__ == "__main__":
    main()
