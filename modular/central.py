import time
import serial
import json
from mqtt_handler import MQTTHandler
from motor_control import MotorControl
from imu_processor import IMUProcessor
from command_processor import CommandProcessor

# MQTT settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPICS = [("golfcart/imu", 0), ("golfcart/buttons", 0)]


def main():
    try:
        # Initialize the serial connection
        uno_serial = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
        print("Serial connection to Uno established.")

        # Initialize components
        motor_control = MotorControl(uno_serial)
        imu_processor = IMUProcessor()
        command_processor = CommandProcessor(motor_control, imu_processor)
        mqtt_handler = MQTTHandler(BROKER_ADDRESS, BROKER_PORT, TOPICS)

        # Attach callback functions
        mqtt_handler.set_callback("golfcart/imu", imu_processor.process_imu_data)
        mqtt_handler.set_callback("golfcart/buttons", command_processor.process_command)

        # Start MQTT loop
        mqtt_handler.start()

        # Store the last known state to avoid unnecessary updates
        last_state = None
        last_imu_data = None

        while True:
            # Get the latest IMU data and command state
            imu_data = imu_processor.get_latest_data()
            current_state = command_processor.get_state()

            # Check if there is a significant change in IMU data or state
            if imu_data != last_imu_data or current_state != last_state:
                # Update motors only when there's a change
                motor_control.update_motors(imu_data, current_state)
                
                # Update the last known state and IMU data
                last_imu_data = imu_data
                last_state = current_state

            # Small sleep to avoid high CPU usage
            time.sleep(0.05)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup resources
        if 'mqtt_handler' in locals():
            mqtt_handler.stop()
        if 'motor_control' in locals():
            motor_control.close()


if __name__ == "__main__":
    main()
