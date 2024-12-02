import json
import time
import paho.mqtt.client as mqtt
import numpy as np
from collections import defaultdict

# MQTT Settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPIC_CAMERA_DETECTIONS = "golfcart/camera/detections"
TOPIC_LIDAR_3D = "golfcart/lidar_3d"
TOPIC_FUSED_DATA = "golfcart/fused_data"

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
mqtt_client.loop_start()

# Data storage
camera_detections = defaultdict(list)  # key: timestamp, value: list of detections
lidar_points = defaultdict(list)        # key: timestamp, value: list of points

def on_camera_detection(client, userdata, msg):
    """Handle camera detection messages."""
    data = json.loads(msg.payload.decode())
    timestamp = data.get("timestamp", time.time())
    detections = data.get("detections", [])
    camera_detections[timestamp].extend(detections)
    print(f"Received camera detections at {timestamp}: {detections}")
    fuse_data(timestamp)

def on_lidar_data(client, userdata, msg):
    """Handle LiDAR 3D data messages."""
    data = json.loads(msg.payload.decode())
    timestamp = data.get("timestamp", time.time())
    points = data.get("data", [])
    lidar_points[timestamp].extend(points)
    print(f"Received LiDAR points at {timestamp}: {len(points)} points")
    fuse_data(timestamp)

def fuse_data(timestamp):
    """Fuse camera detections with LiDAR points based on timestamp."""
    # Define a time window for association (e.g., 0.1 seconds)
    window = 0.1
    # Find matching timestamps within the window
    matching_timestamps = [t for t in lidar_points.keys() if abs(t - timestamp) <= window]
    if not matching_timestamps:
        return
    for t in matching_timestamps:
        detections = camera_detections.get(t, [])
        points = lidar_points.get(t, [])
        if detections and points:
            # Example: Assign nearest LiDAR point to each detection
            fused_detections = []
            for det in detections:
                # Assume det has 'bbox' with (x, y, w, h)
                bbox = det.get("bbox", [0,0,0,0])
                center_x = bbox[0] + bbox[2] / 2
                center_y = bbox[1] + bbox[3] / 2
                # Simple projection: map image center to LiDAR points
                # Needs proper calibration in real implementation
                nearest_point = find_nearest_lidar_point(center_x, center_y, points)
                if nearest_point:
                    fused = {
                        "label": det.get("label", "unknown"),
                        "confidence": det.get("confidence", 0.0),
                        "bbox": bbox,
                        "position_3d": nearest_point
                    }
                    fused_detections.append(fused)
            # Publish fused data
            if fused_detections:
                fused_packet = {
                    "timestamp": t,
                    "detections": fused_detections
                }
                mqtt_client.publish(TOPIC_FUSED_DATA, json.dumps(fused_packet))
                print(f"Published fused data at {t}: {fused_detections}")
        # Clean up old data
        del lidar_points[t]
        del camera_detections[t]

def find_nearest_lidar_point(x, y, points):
    """Find the nearest LiDAR point to the given image center (placeholder)."""
    # Placeholder implementation: return a random point
    if points:
        return points[0]  # Replace with actual nearest point calculation
    return None

def main():
    # Subscribe to necessary topics
    mqtt_client.subscribe(TOPIC_CAMERA_DETECTIONS)
    mqtt_client.subscribe(TOPIC_LIDAR_3D)

    # Assign callbacks
    mqtt_client.message_callback_add(TOPIC_CAMERA_DETECTIONS, on_camera_detection)
    mqtt_client.message_callback_add(TOPIC_LIDAR_3D, on_lidar_data)

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Fused Data Node Shutting Down...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    main()
