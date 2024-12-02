import serial
import time

# Replace with your actual port
port = '/dev/ttyUSB0'
baud_rate = 115200

# Define calibration offsets
accel_offsets = {'ax': -31.99, 'ay': -32.00, 'az': -0.01}
gyro_offsets = {'wx': 0.0, 'wy': 0.0, 'wz': 0.0}
angle_offsets = {'roll': -359.72, 'pitch': -0.60, 'yaw': -31.72}

try:
    # Open the serial connection
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Connected to {port} at {baud_rate} baud.")
    
    # Allow time for the device to initialize
    time.sleep(2)

    while True:
        # Read a full packet (20 bytes)
        if ser.in_waiting >= 20:
            data = ser.read(20)
            
            # Check packet header and flag
            if data[0] == 0x55 and data[1] == 0x61:
                # Unpack data as unsigned bytes
                axL, axH, ayL, ayH, azL, azH = data[2:8]
                wxL, wxH, wyL, wyH, wzL, wzH = data[8:14]
                RollL, RollH, PitchL, PitchH, YawL, YawH = data[14:20]

                # Calculate acceleration (in g) and apply offsets
                ax = ((axH << 8) | axL) / 32768.0 * 16 + accel_offsets['ax']
                ay = ((ayH << 8) | ayL) / 32768.0 * 16 + accel_offsets['ay']
                az = ((azH << 8) | azL) / 32768.0 * 16 + accel_offsets['az']

                # Calculate angular velocity (in ?/s) and apply offsets
                wx = ((wxH << 8) | wxL) / 32768.0 * 2000 + gyro_offsets['wx']
                wy = ((wyH << 8) | wyL) / 32768.0 * 2000 + gyro_offsets['wy']
                wz = ((wzH << 8) | wzL) / 32768.0 * 2000 + gyro_offsets['wz']

                # Calculate angles (in ?) and apply offsets
                roll = ((RollH << 8) | RollL) / 32768.0 * 180 + angle_offsets['roll']
                pitch = ((PitchH << 8) | PitchL) / 32768.0 * 180 + angle_offsets['pitch']
                yaw = ((YawH << 8) | YawL) / 32768.0 * 180 + angle_offsets['yaw']

                # Print calibrated values
                print(f"Acceleration (g): ax={ax:.2f}, ay={ay:.2f}, az={az:.2f}")
                print(f"Angular Velocity (?/s): wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}")
                print(f"Angle (?): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
                print("-" * 50)

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")
