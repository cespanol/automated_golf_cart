# draft of 2d lidar rotated with servo to create 3d scan

import serial
import time
import json
import paho.mqtt.client as mqtt
import numpy as np
from threading import Thread
from queue import Queue, Empty
import dash
from dash import dcc, html
from dash.dependencies import Output, Input
import plotly.graph_objs as go
import RPi.GPIO as GPIO

# ======================== Configuration ========================

# MQTT Settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPIC_LIDAR = "golfcart/lidar_3d"

# LiDAR Settings
LIDAR_PORT = "/dev/ttyUSB0"  # Update with your LiDAR's USB port
BAUD_RATE = 115200
FRAME_ID = "lidar_frame"

# Servo Settings
SERVO_PIN = 18  # GPIO pin connected to the servo
SERVO_MIN_DUTY = 2.5  # Minimum duty cycle for servo
SERVO_MAX_DUTY = 12.5  # Maximum duty cycle for servo
SERVO_STEP = 1  # Degrees per step
SERVO_DELAY = 0.02  # Delay between servo steps (seconds)

# Rotation Parameters
ROTATION_ANGLE_MIN = 0
ROTATION_ANGLE_MAX = 180
ROTATION_STEP = 1  # Degrees
ROTATION_DELAY = 0.02  # Delay between steps

# ======================== MQTT Initialization ========================

mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
mqtt_client.loop_start()

def publish_lidar_data(data):
    """Publish LiDAR data to MQTT."""
    message = json.dumps(data)
    mqtt_client.publish(TOPIC_LIDAR, message)

# ======================== LiDAR Data Processing ========================

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

def convert_to_cartesian(angle, distance, servo_angle):
    """Convert polar coordinates with servo angle to Cartesian (x, y, z)."""
    # Total angle combines LiDAR scan angle and servo rotation
    total_angle = (angle + servo_angle) % 360
    angle_rad = np.deg2rad(total_angle)
    # Assuming servo rotates around the Z-axis
    x = distance * np.cos(angle_rad)
    y = distance * np.sin(angle_rad)
    z = servo_angle  # Or use servo_angle converted to radians if needed
    return x, y, z

def lidar_serial_thread(lidar_serial, data_queue, servo_controller):
    """Thread to read data from the LiDAR and pass to the main thread via a queue."""
    while True:
        try:
            raw_data = lidar_serial.read(1000)  # Adjust based on your LiDAR
            if raw_data:
                lidar_data = parse_lidar_data(raw_data)
                cartesian_points = []
                servo_angle = servo_controller.get_current_angle()

                for point in lidar_data:
                    # Convert to 3D Cartesian and add to the list
                    x, y, z = convert_to_cartesian(point["angle"], point["distance"], servo_angle)
                    cartesian_points.append([x, y, z])

                # Debug: Print number of points
                print(f"Number of Cartesian points at servo angle {servo_angle}: {len(cartesian_points)}")

                # Pass the updated points to the visualization queue
                if cartesian_points:
                    data_queue.put(cartesian_points)

                # Publish MQTT data
                data_packet = {
                    "frame_id": FRAME_ID,
                    "timestamp": time.time(),
                    "servo_angle": servo_angle,
                    "data": lidar_data
                }
                publish_lidar_data(data_packet)
        except Exception as e:
            print(f"LiDAR thread error: {e}")
            break

# ======================== Servo Control ========================

class ServoController:
    def __init__(self, pin):
        self.pin = pin
        self.current_angle = ROTATION_ANGLE_MIN
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz servo
        self.pwm.start(self.angle_to_duty(self.current_angle))
        self.lock = Thread.Lock()

    def angle_to_duty(self, angle):
        """Convert angle to duty cycle."""
        return SERVO_MIN_DUTY + (SERVO_MAX_DUTY - SERVO_MIN_DUTY) * (angle / 180.0)

    def move_to_angle(self, angle):
        """Move servo to specified angle."""
        duty = self.angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.05)  # Allow servo to move
        self.pwm.ChangeDutyCycle(0)  # Stop sending signal to prevent jitter

    def rotate_scan(self, data_queue):
        """Rotate servo and collect data."""
        try:
            while True:
                for angle in range(ROTATION_ANGLE_MIN, ROTATION_ANGLE_MAX + 1, ROTATION_STEP):
                    self.move_to_angle(angle)
                    self.current_angle = angle
                    time.sleep(ROTATION_DELAY)
        except KeyboardInterrupt:
            print("Servo rotation interrupted.")

    def get_current_angle(self):
        """Get current servo angle."""
        return self.current_angle

    def cleanup(self):
        """Cleanup GPIO."""
        self.pwm.stop()
        GPIO.cleanup()

# ======================== Data Visualization ========================

def visualize_point_cloud(data_queue):
    """Visualize 3D point cloud using Plotly Dash with dynamic updates."""
    # Initialize Dash app
    app = dash.Dash(__name__)
    app.layout = html.Div(style={'backgroundColor': '#111111', 'color': '#7FDBFF'}, children=[
        html.H1('Real-Time 3D LiDAR Point Cloud', style={'textAlign': 'center'}),
        dcc.Graph(id='live-point-cloud'),
        dcc.Interval(
            id='interval-component',
            interval=100,  # Update every 100 milliseconds
            n_intervals=0
        )
    ])

    @app.callback(Output('live-point-cloud', 'figure'),
                  [Input('interval-component', 'n_intervals')])
    def update_graph(n):
        """Update the point cloud graph with new data from the queue."""
        cartesian_points = []
        try:
            # Retrieve all points currently in the queue
            while True:
                points = data_queue.get_nowait()
                cartesian_points.extend(points)
        except Empty:
            pass  # No new data

        if cartesian_points:
            x_vals, y_vals, z_vals = zip(*cartesian_points)
        else:
            x_vals, y_vals, z_vals = [], [], []

        # Create the Scatter3d plot
        scatter = go.Scatter3d(
            x=x_vals,
            y=y_vals,
            z=z_vals,
            mode='markers',
            marker=dict(
                size=2,
                color='lime',
                opacity=0.8
            )
        )

        layout = go.Layout(
            title='LiDAR 3D Point Cloud',
            scene=dict(
                xaxis=dict(title='X (mm)', range=[-12000, 12000]),
                yaxis=dict(title='Y (mm)', range=[-12000, 12000]),
                zaxis=dict(title='Z (Servo Angle)', range=[ROTATION_ANGLE_MIN, ROTATION_ANGLE_MAX]),
            ),
            showlegend=False,
            width=800,
            height=800,
            plot_bgcolor='black',
            paper_bgcolor='black'
        )

        figure = go.Figure(data=[scatter], layout=layout)
        return figure

    app.run_server(host='0.0.0.0', port=8051, debug=False, use_reloader=False)

# ======================== Main Function ========================

def main():
    # Initialize a thread-safe queue for passing data
    data_queue = Queue()

    # Initialize and start Servo Controller
    servo_controller = ServoController(SERVO_PIN)

    # Initialize and start LiDAR thread
    try:
        lidar_serial = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error connecting to LiDAR: {e}")
        return

    lidar_thread = Thread(target=lidar_serial_thread, args=(lidar_serial, data_queue, servo_controller), daemon=True)
    lidar_thread.start()

    # Start Servo Rotation in a separate thread
    servo_thread = Thread(target=servo_controller.rotate_scan, args=(data_queue,), daemon=True)
    servo_thread.start()

    # Start Visualization in another thread
    viz_thread = Thread(target=visualize_point_cloud, args=(data_queue,), daemon=True)
    viz_thread.start()

    try:
        while True:
            time.sleep(1)  # Keep main thread alive
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        servo_controller.cleanup()
        lidar_serial.close()
        mqtt_client.disconnect()
        mqtt_client.loop_stop()

if __name__ == "__main__":
    main()
