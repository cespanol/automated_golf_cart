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

# ======================== Configuration ========================

# MQTT Settings
BROKER_ADDRESS = "192.168.1.246"
BROKER_PORT = 1883
TOPIC_LIDAR = "golfcart/lidar"

# LiDAR Settings
LIDAR_PORT = "/dev/ttyUSB0"  # Update with your LiDAR's USB port
BAUD_RATE = 230400  # Updated baud rate
FRAME_ID = "lidar_frame"

# Initialize MQTT client
mqtt_client = mqtt.Client()
try:
    mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
mqtt_client.loop_start()

def publish_lidar_data(data):
    """Publish LiDAR data to MQTT."""
    try:
        message = json.dumps(data)
        mqtt_client.publish(TOPIC_LIDAR, message)
    except Exception as e:
        print(f"Failed to publish MQTT message: {e}")

def parse_lidar_data(raw_data):
    """Parse raw LiDAR binary data into distance and angle readings."""
    parsed_data = []
    try:
        for i in range(0, len(raw_data), 5):  # Process in 5-byte packets
            if i + 4 < len(raw_data):  # Ensure a complete packet
                angle_raw = int.from_bytes(raw_data[i:i+2], byteorder='little', signed=False)
                distance_raw = int.from_bytes(raw_data[i+2:i+4], byteorder='little', signed=False)
                quality = raw_data[i+4]  # Quality byte

                angle = (angle_raw / 100.0) % 360  # Convert to degrees and wrap around
                distance = distance_raw / 1000.0    # Convert to meters

                # Filter invalid points
                if 0 < distance <= 12.0 and quality > 0:
                    parsed_data.append({"angle": angle, "distance": distance, "quality": quality})
    except Exception as e:
        print(f"Error parsing LiDAR data: {e}")
    return parsed_data

def convert_to_cartesian(angle, distance):
    """Convert polar coordinates (angle, distance) to Cartesian (x, y) in meters."""
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
                    cartesian_points.append([x, y, point["quality"]])

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

def start_lidar_thread(data_queue):
    """Initialize and start the LiDAR serial thread."""
    try:
        lidar_serial = serial.Serial(LIDAR_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to LiDAR on {LIDAR_PORT} at {BAUD_RATE} baud.")
        thread = Thread(target=lidar_serial_thread, args=(lidar_serial, data_queue), daemon=True)
        thread.start()
        return lidar_serial
    except serial.SerialException as e:
        print(f"Error connecting to LiDAR: {e}")
        return None  # LiDAR not connected

# ======================== Visualization Setup ========================

# Initialize a thread-safe queue for passing data
data_queue = Queue()

# Start the LiDAR thread
lidar_serial = start_lidar_thread(data_queue)

# Initialize Dash app
app = dash.Dash(__name__)
app.layout = html.Div(style={'backgroundColor': '#111111', 'color': '#7FDBFF'}, children=[
    html.H1('Real-Time LiDAR Point Cloud', style={'textAlign': 'center'}),
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
    distances = []
    qualities = []
    try:
        # Retrieve all points currently in the queue
        while True:
            points = data_queue.get_nowait()
            for x, y, quality in points:
                cartesian_points.append([x, y])
                distances.append(np.sqrt(x**2 + y**2))
                qualities.append(quality)
    except Empty:
        pass  # No new data

    if cartesian_points:
        x_vals, y_vals = zip(*cartesian_points)
    else:
        x_vals, y_vals = [], []

    # Create the Scatter plot with color based on quality
    scatter = go.Scatter(
        x=x_vals,
        y=y_vals,
        mode='markers',
        marker=dict(
            size=3,
            color=qualities,  # Color by quality
            colorscale='Viridis',
            colorbar=dict(title='Quality'),
            opacity=0.8
        )
    )

    # Define fixed plot range based on LiDAR's maximum range
    fixed_range = 12  # meters
    min_x, max_x = -fixed_range, fixed_range
    min_y, max_y = -fixed_range, fixed_range

    # Add reference lines at origin
    shapes = [
        dict(
            type="line",
            x0=min_x,
            y0=0,
            x1=max_x,
            y1=0,
            line=dict(color="gray", width=1)
        ),
        dict(
            type="line",
            x0=0,
            y0=min_y,
            x1=0,
            y1=max_y,
            line=dict(color="gray", width=1)
        )
    ]

    layout = go.Layout(
        title='LiDAR Point Cloud (Meters)',
        xaxis=dict(title='X (m)', range=[min_x, max_x], zeroline=False),
        yaxis=dict(title='Y (m)', range=[min_y, max_y], zeroline=False),
        showlegend=False,
        width=800,
        height=800,
        plot_bgcolor='black',
        paper_bgcolor='black',
        shapes=shapes
    )

    figure = go.Figure(data=[scatter], layout=layout)
    return figure

def run_dash():
    """Run the Dash app."""
    app.run_server(host='0.0.0.0', port=8050, debug=False, use_reloader=False)

def main():
    # Start Dash app in the main thread
    dash_thread = Thread(target=run_dash, daemon=True)
    dash_thread.start()

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up
        if lidar_serial is not None:
            lidar_serial.close()
        mqtt_client.disconnect()
        mqtt_client.loop_stop()

if __name__ == "__main__":
    main()
