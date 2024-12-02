import asyncio
import time
from bleak import BleakScanner, BleakClient
import serial

# Serial settings for Uno
UNO_PORT = "/dev/ttyUSB0"  # Update based on your system
UNO_BAUD_RATE = 9600

# BLE settings
NANO_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef1"
NANO_ADDRESS = "D4:D4:DA:4F:7F:82"
WT901_ADDRESS = "DA:10:A2:CE:17:23"
WT901_CHARACTERISTIC_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

# Calibration offsets
accel_offsets = {'ax': -31.99, 'ay': -32.00, 'az': -0.01}
gyro_offsets = {'wx': 0.0, 'wy': 0.0, 'wz': 0.0}
angle_offsets = {'roll': -359.72, 'pitch': -0.60, 'yaw': -31.72}

# Motor states
STATE_STOP = -1
STATE_FRONT = 1
STATE_BACK = 2
STATE_LEFT = 3
STATE_RIGHT = 4

# Global variables
uno_serial = None
imu_data = 0  # Current deviation data for adjustments
latest_wt901_data = {}  # Dictionary to hold the most recent IMU data
current_state = STATE_STOP
base_speed = 150
update_interval = 0.1  # Update every 100ms
last_update_time = time.time()
baseline_yaw = 0  # Dynamic baseline for yaw


# Serial initialization
def initialize_uno():
    """Initialize serial connection to Uno."""
    try:
        serial_connection = serial.Serial(UNO_PORT, UNO_BAUD_RATE, timeout=1)
        print("Serial connection to Uno established.")
        return serial_connection
    except Exception as e:
        print(f"Error initializing Uno serial: {e}")
        exit()

# Send motor commands to the Uno
def send_motor_command(direction, left_pwm, right_pwm):
    """Send motor command to Uno."""
    global uno_serial
    message = f"{direction},{left_pwm},{right_pwm}\n"
    try:
        uno_serial.write(message.encode())
        print(f"Sent to Uno: {message.strip()}")
    except Exception as e:
        print(f"Error sending motor command to Uno: {e}")

# Periodic motor updates based on IMU feedback
def update_motors():
    """Periodically adjust motor commands based on IMU feedback."""
    global imu_data, latest_wt901_data, current_state, last_update_time

    now = time.time()
    if now - last_update_time < update_interval:
        return  # Only update at the defined interval

    if current_state == STATE_STOP:
        return  # No motor updates if the cart is stopped

    if 'yaw' not in latest_wt901_data:
        print("Yaw data missing. Skipping motor update.")
        return

    # Calculate yaw error relative to the baseline yaw
    desired_yaw = baseline_yaw  # Use dynamic baseline yaw
    yaw_error = latest_wt901_data['yaw'] - desired_yaw

    # Wrap yaw error within �180 degrees for consistent calculations
    if yaw_error > 180:
        yaw_error -= 360
    elif yaw_error < -180:
        yaw_error += 360

    # Flip yaw error for reverse movement
    if current_state == STATE_BACK:
        yaw_error *= -1

    k_p = 0.2  # Proportional gain
    max_adjustment = 50  # Limit maximum adjustment to avoid extreme values
    dead_zone = 5  # Ignore yaw errors within �5 degrees

    # Ignore small yaw errors
    if abs(yaw_error) <= dead_zone:
        yaw_error = 0

    # Calculate adjustment with clamping
    adjustment = max(-max_adjustment, min(max_adjustment, k_p * yaw_error))

    # Calculate PWM values with a minimum threshold
    min_pwm = 100  # Ensure motors receive enough power to avoid stalling
    left_pwm = max(min_pwm, base_speed - adjustment)
    right_pwm = max(min_pwm, base_speed + adjustment)

    # Clamp PWM values to valid range
    left_pwm = max(0, min(255, left_pwm))
    right_pwm = max(0, min(255, right_pwm))

    # Send the motor command
    send_motor_command(current_state, int(left_pwm), int(right_pwm))

    # Log adjustments for debugging
    print(f"[Motor Update] State: {current_state}, Yaw Error: {yaw_error:.2f}, "
          f"Baseline Yaw: {baseline_yaw:.2f}, Current Yaw: {latest_wt901_data['yaw']:.2f}, "
          f"Left PWM: {left_pwm:.2f}, Right PWM: {right_pwm:.2f}")
    print("-" * 50)

    last_update_time = now


# Update motor state based on commands
def process_command(command):
    """Update motor state based on user input."""
    global current_state, baseline_yaw, latest_wt901_data

    if command == "w":
        # Set baseline yaw when switching to forward motion
        if current_state != STATE_FRONT and 'yaw' in latest_wt901_data:
            baseline_yaw = latest_wt901_data['yaw']
        current_state = STATE_FRONT
    elif command == "s":
        # Set baseline yaw when switching to reverse motion
        if current_state != STATE_BACK and 'yaw' in latest_wt901_data:
            baseline_yaw = latest_wt901_data['yaw']
        current_state = STATE_BACK
    elif command == "a":
        current_state = STATE_LEFT
    elif command == "d":
        current_state = STATE_RIGHT
    elif command == "x":
        current_state = STATE_STOP
    else:
        print(f"Unknown command: {command}")



# Handle Nano BLE notifications
async def handle_nano_notifications(client):
    """Handle notifications from the Nano BLE device."""
    print("Listening for commands from Nano...")
    try:
        def nano_notification_handler(sender, data):
            """Process Nano commands."""
            command = data.decode('utf-8').strip()
            process_command(command)

        await client.start_notify(NANO_CHARACTERISTIC_UUID, nano_notification_handler)
        await asyncio.Future()  # Keeps the function alive indefinitely
    except asyncio.CancelledError:
        print("Nano notification handling cancelled.")
    finally:
        await client.stop_notify(NANO_CHARACTERISTIC_UUID)

# Handle WT901 BLE notifications
async def handle_wt901_notifications(client):
    """Handle notifications from the WT901 IMU device."""
    print("Listening for IMU data from WT901...")
    try:
        def wt901_notification_handler(sender, data):
            """Process IMU data from WT901."""
            global imu_data, latest_wt901_data

            if len(data) >= 20:
                # Extract calibrated data
                ax = ((data[3] << 8 | data[2]) / 32768.0 * 16) + accel_offsets['ax']
                ay = ((data[5] << 8 | data[4]) / 32768.0 * 16) + accel_offsets['ay']
                az = ((data[7] << 8 | data[6]) / 32768.0 * 16) + accel_offsets['az']
                wx = ((data[9] << 8 | data[8]) / 32768.0 * 2000) + gyro_offsets['wx']
                wy = ((data[11] << 8 | data[10]) / 32768.0 * 2000) + gyro_offsets['wy']
                wz = ((data[13] << 8 | data[12]) / 32768.0 * 2000) + gyro_offsets['wz']
                roll = ((data[15] << 8 | data[14]) / 32768.0 * 180) + angle_offsets['roll']
                pitch = ((data[17] << 8 | data[16]) / 32768.0 * 180) + angle_offsets['pitch']
                yaw = ((data[19] << 8 | data[18]) / 32768.0 * 180) + angle_offsets['yaw']

                latest_wt901_data.update({'ax': ax, 'ay': ay, 'az': az,
                                          'wx': wx, 'wy': wy, 'wz': wz,
                                          'roll': roll, 'pitch': pitch, 'yaw': yaw})
                imu_data = roll
            else:
                print("Incomplete data received from WT901!")

        await client.start_notify(WT901_CHARACTERISTIC_UUID, wt901_notification_handler)
        await asyncio.Future()  # Keep the function alive
    except asyncio.CancelledError:
        print("WT901 notification handling cancelled.")
    finally:
        if client.is_connected:
            await client.stop_notify(WT901_CHARACTERISTIC_UUID)

# Periodic motor update task
async def periodic_motor_updates():
    while True:
        update_motors()
        await asyncio.sleep(0.01)  # Update motors every 10ms



# Main function
async def main():
    global uno_serial
    uno_serial = initialize_uno()

    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    nano_device = None
    wt901_device = None

    for d in devices:
        if d.address == NANO_ADDRESS:
            nano_device = d
        if d.address == WT901_ADDRESS:
            wt901_device = d

    if not nano_device or not wt901_device:
        print("Nano or WT901 not found.")
        return

    print(f"Found Nano: {nano_device.name}, {nano_device.address}")
    print(f"Found WT901: {wt901_device.name}, {wt901_device.address}")

    try:
        async with BleakClient(nano_device.address) as nano_client, BleakClient(wt901_device.address) as wt901_client:
            print("Connected to Nano and WT901.")
            await asyncio.gather(
                handle_nano_notifications(nano_client),
                handle_wt901_notifications(wt901_client),
                periodic_motor_updates()
            )
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(main())