import serial
import time

def send_command(ser, direction, left_pwm, right_pwm):
    """
    Sends a motor command to the Arduino Uno.

    Args:
        ser (serial.Serial): Serial connection to Uno.
        direction (int): Motor direction (-1 for stop, 1 for forward, etc.).
        left_pwm (int): Left motor PWM speed (0-255).
        right_pwm (int): Right motor PWM speed (0-255).
    """
    command = f"{direction},{left_pwm},{right_pwm}\n"
    ser.write(command.encode())
    print(f"Sent: {command.strip()}")
    time.sleep(2)  # Allow time to observe motor behavior


def main():
    # Connect to the Arduino Uno
    try:
        ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=1)
        print("Connected to Arduino Uno.")
    except serial.SerialException as e:
        print(f"Error: {e}")
        return

    # Test motor commands
    try:
        # Test Forward
        send_command(ser, 1, 66, 66)
        # Test Left
        send_command(ser, 3, 80, 80)
        # Test Right
        send_command(ser, 4, 90, 90)
        # Test Reverse
        send_command(ser, 2, 90, 90)
        # Test Stop
        send_command(ser, -1, 0, 0)
        # send_command(ser, 1, 66, 66)
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        ser.close()
        print("Test complete. Connection closed.")

if __name__ == "__main__":
    main()
