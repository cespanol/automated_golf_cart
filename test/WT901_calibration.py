import serial
import time

# Replace with your actual port
port = '/dev/rfcomm0'  # or '/dev/ttyUSB0' for USB connection
baud_rate = 115200
ser = serial.Serial(port, baud_rate, timeout=1)

def send_calibration_command(command):
    ser.write(command)
    time.sleep(0.5)  # Short delay to process command
    while ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        print("Response:", response)

def read_calibration_values():
    # Adjust these commands if needed to read back calibration values
    print("Reading calibration offset values...")
    ser.write(b'\xFF\xAA\x27\x00\x00')  # Example command to read offsets
    time.sleep(1)
    while ser.in_waiting > 0:
        response = ser.readline().decode('utf-8').strip()
        print("Calibration Offset Response:", response)

try:
    print("Starting Calibration for WT901...")

    # 1. Acceleration Calibration
    print("Starting Acceleration Calibration...")
    print("Place the sensor level and keep it steady.")
    time.sleep(2)
    send_calibration_command(b'\xFF\xAA\x01\x01\x00')  # Acceleration calibration command
    print("Acceleration Calibration done. Move to next step.")

    # 2. Magnetic Field Calibration (Tilt sensor in all directions)
    print("Starting Magnetic Calibration...")
    print("Rotate the sensor around all axes for 1 minute.")
    send_calibration_command(b'\xFF\xAA\x01\x07\x00')  # Start magnetic calibration
    time.sleep(60)  # Rotate sensor in all directions
    send_calibration_command(b'\xFF\xAA\x01\x07\x01')  # End magnetic calibration
    print("Magnetic Calibration done.")

    # 3. Save Calibration
    print("Saving Calibration Data...")
    send_calibration_command(b'\xFF\xAA\x00\x00\x00')  # Save command
    print("Calibration data saved.")

    # 4. Read Calibration Offsets
    read_calibration_values()

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    ser.close()
    print("Serial connection closed.")
