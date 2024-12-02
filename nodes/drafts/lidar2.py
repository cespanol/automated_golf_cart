import serial
import time
import json
import paho.mqtt.client as mqtt
import numpy as np
import open3d as o3d
from threading import Thread
from queue import Queue

# MQTT Settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPIC = "golfcart/lidar"

# LiDAR Settings
# LIDAR_PORT = "/dev/tty.usbserial-0001"  
LIDAR_PORT = "/dev/ttyUSB0"  # Update with your LiDAR's USB port

BAUD_RATE = 115200
FRAME_ID = "lidar_frame"

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)

def publish_lidar_data(data):
    """Publish LiDAR data to MQTT."""
    message = json.dumps(data)
    mqtt_client.publish(TOPIC, message)
    # Commented out to reduce console clutter
    # print(f"Published: {message}")

def parse_lidar_data(raw_data):
    """Parse raw LiDAR binary data into distance and angle readings."""
    parsed_data = []
    try:
        for i in range(0, len(raw_data), 5):  # Process in 5-byte packets
            if i + 4 < len(raw_data):  # Ensure a complete packet
                angle_raw = int.from_bytes(raw_data[i:i+2], byteorder='little')
                distance_raw = int.from_bytes(raw_data[i+2:i+4], byteorder='little')
                quality = raw_data[i+4]  # Quality byte

                angle = (angle_raw / 100.0) % 360  # Scale and wrap angle
                distance = distance_raw  # Distance in millimeters

                # Filter invalid points
                if 0 < distance <= 12000 and quality > 0:
                    parsed_data.append({"angle": angle, "distance": distance, "quality": quality})
    except Exception as e:
        print(f"Error parsing LiDAR data: {e}")
    return parsed_data

def convert_to_cartesian(angle, distance):
    """Convert polar coordinates (angle, distance) to Cartesian (x, y)."""
    angle_rad = np.deg2rad(angle)
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    return x, y

def lidar_serial_thread(lidar_serial, data_queue):
    """Thread to read data from the LiDAR and pass to the main thread via a queue."""
    while True:
        try:
            raw_data = lidar_serial.read(1000)  # Adjust based on your LiDAR
            if raw_data:
                lidar_data = parse_lidar_data(raw_data)
                cartesian_points = []
                for point in lidar_data:
                    # Convert to Cartesian and add to the list
                    x, y = convert_to_cartesian(point["angle"], point["distance"])
                    cartesian_points.append([x, y, 0.0])  # Assume z=0 for 2D LiDAR

                # Debug: Print number of points
                print(f"Number of Cartesian points: {len(cartesian_points)}")

                # Pass the updated points to the visualization queue
                if cartesian_points:
                    data_queue.put(cartesian_points)

                # Publish MQTT data
                data_packet = {
                    "frame_id": FRAME_ID,
                    "timestamp": time.time(),
                    "data": lidar_data
                }
                publish_lidar_data(data_packet)
        except Exception as e:
            print(f"LiDAR thread error: {e}")
            break

def visualize_point_cloud(data_queue):
    """Visualize point cloud using Open3D in the main thread."""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='LiDAR Point Cloud', width=800, height=600)
    pcd = o3d.geometry.PointCloud()

    # Add coordinate axes for reference
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(axis)

    # Wait for initial data before adding the point cloud
    while True:
        points = np.array(data_queue.get()) / 1000.0  # Convert from mm to meters
        if points.size > 0:
            break
    print(f"Initial queue points size: {len(points)}")  # Debug

    # Initialize the point cloud with initial data
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Set point colors to be fluorescent (e.g., bright neon green)
    colors = np.array([[0.0, 1.0, 0.0]] * len(points))  # RGB value for neon green
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vis.add_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    # Customize visualization options
    render_option = vis.get_render_option()
    render_option.point_size = 5.0  # Increase point size for better visibility
    render_option.background_color = np.asarray([0, 0, 0])  # Set background to black

    try:
        while True:
            # Block until data is available
            points = np.array(data_queue.get()) / 1000.0  # Convert from mm to meters
            print(f"Queue points size: {len(points)}")  # Debug
            if points.size > 0:
                pcd.points = o3d.utility.Vector3dVector(points)
                
                # Update the colors again for new points (keep them fluorescent)
                colors = np.array([[0.0, 1.0, 0.0]] * len(points))  # RGB value for neon green
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                vis.update_geometry(pcd)  # Apply changes

            vis.poll_events()
            vis.update_renderer()
    except KeyboardInterrupt:
        print("Exiting visualization.")
    finally:
        vis.destroy_window()

def main():
    # Thread-safe queue for passing data
    data_queue = Queue()

    # Try to open serial port and start LiDAR thread
    try:
        lidar_serial = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud.")
        # Start LiDAR serial thread
        thread = Thread(target=lidar_serial_thread, args=(lidar_serial, data_queue), daemon=True)
        thread.start()
    except serial.SerialException as e:
        print(f"Error connecting to LiDAR: {e}")
        lidar_serial = None  # LiDAR not connected

    # Start mock data thread only if LiDAR is not connected
    # if lidar_serial is None:
    #     def mock_lidar_data():
    #         while True:
    #             # Generate simulated LiDAR scan data in millimeters
    #             angles = np.linspace(0, 360, 360, endpoint=False)
    #             distances = 5000 + 1000 * np.sin(np.deg2rad(angles * 4))  # Simulate a pattern
    #             cartesian_points = []
    #             for angle, distance in zip(angles, distances):
    #                 x, y = convert_to_cartesian(angle, distance)
    #                 cartesian_points.append([x, y, 0.0])
    #             data_queue.put(cartesian_points)
    #             time.sleep(0.1)

    #     Thread(target=mock_lidar_data, daemon=True).start()
    else:
        # Only use LiDAR data
        pass

    # Start Open3D visualization
    visualize_point_cloud(data_queue)

    # Close serial port on exit
    if lidar_serial is not None:
        lidar_serial.close()
    mqtt_client.disconnect()

if __name__ == "__main__":
    main()
