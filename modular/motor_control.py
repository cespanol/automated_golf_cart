import serial
import time


class MotorControl:
    """
    A class to handle motor control by sending commands to an Arduino Uno.
    """

    # Direction constants for the Uno
    DIRECTION_MAP = {
        "STOP": -1,
        "FORWARD": 1,
        "BACKWARD": 2,
        "LEFT": 3,
        "RIGHT": 4
    }

    def __init__(self, serial_connection, baudrate=9600, base_speed=66, command_timeout=1.0):
        """
        Initialize the MotorControl class.

        Args:
            serial_connection (str or serial.Serial): The serial port to which the Uno is connected
                (e.g., "/dev/ttyUSB0") or an existing `serial.Serial` instance.
            baudrate (int): Baud rate for serial communication.
            base_speed (int): Base PWM speed for the motors.
            command_timeout (float): Timeout in seconds for communication before stopping motors.
        """
        if isinstance(serial_connection, str):
            try:
                self.uno_serial = serial.Serial(serial_connection, baudrate, timeout=1)
                print(f"Serial connection to Uno established on {serial_connection}.")
            except serial.SerialException as e:
                print(f"Error initializing serial connection: {e}")
                raise
        elif isinstance(serial_connection, serial.Serial):
            self.uno_serial = serial_connection
            print("Using provided serial connection to Uno.")
        else:
            raise ValueError('"serial_connection" must be a string or an instance of serial.Serial.')

        self.base_speed = base_speed
        self.command_timeout = command_timeout
        self.last_command_time = time.time()
        self.baseline_yaw = None
        self.last_transition_time = 0
        self.filtered_yaw = 0
        self.alpha = 0.1  # Low-pass filter coefficient for yaw smoothing

    def send_motor_command(self, direction, left_pwm, right_pwm):
        """
        Send a motor command to the Uno.

        Args:
            direction (int): The direction of the motor (-1 for stop, 1 for forward, etc.).
            left_pwm (int): PWM value for the left motor (0-80).
            right_pwm (int): PWM value for the right motor (0-80).
        """
        try:
            left_pwm = max(0, min(80, left_pwm))
            right_pwm = max(0, min(80, right_pwm))
            command = f"{direction},{left_pwm},{right_pwm}\n"
            self.uno_serial.write(command.encode())
            print(f"Sent to Uno: {command.strip()}")
            self.last_command_time = time.time()
        except Exception as e:
            print(f"Error sending motor command to Uno: {e}")

    def set_baseline_yaw(self, yaw, stabilization_time=0.5):
        """
        Set the baseline yaw for directional corrections with stabilization.

        Args:
            yaw (float): Current yaw value to set as baseline.
            stabilization_time (float): Time in seconds to stabilize yaw before setting baseline.
        """
        time.sleep(stabilization_time)  # Allow the system to stabilize
        self.baseline_yaw = yaw
        self.last_transition_time = time.time()
        print(f"Baseline yaw set to: {yaw}")

    def update_yaw(self, raw_yaw, max_rate=10.0):
        """
        Update the filtered yaw with a rate-of-change filter.

        Args:
            raw_yaw (float): Current yaw reading from the IMU.
            max_rate (float): Maximum allowable rate of change for yaw per update.
        """
        delta_yaw = raw_yaw - self.filtered_yaw
        if abs(delta_yaw) > max_rate:
            delta_yaw = max_rate if delta_yaw > 0 else -max_rate
        self.filtered_yaw += delta_yaw

    def dynamic_kp_adjustment(self, current_state):
        """
        Dynamically adjust Kp based on state and yaw stability.

        Args:
            current_state (str): Current state of the system (e.g., "FORWARD").
        """
        if current_state in ["FORWARD", "BACKWARD"]:
            return 0.1  # Lower Kp for straight-line motion
        else:
            return 0.2  # Higher Kp for turns

    def update_motors(self, imu_data, current_state):
        """
        Update motor commands based on IMU data and the current state.

        Args:
            imu_data (dict): IMU data containing yaw information.
            current_state (str): Current state of the system (e.g., "FORWARD").
        """
        direction = self.DIRECTION_MAP.get(current_state, -1)

        if direction == -1:
            self.send_motor_command(-1, 0, 0)
            return

        yaw = imu_data.get("yaw", 0)
        self.update_yaw(yaw)

        if self.baseline_yaw is None:
            self.set_baseline_yaw(self.filtered_yaw)

        yaw_error = self.filtered_yaw - self.baseline_yaw
        if yaw_error > 180:
            yaw_error -= 360
        elif yaw_error < -180:
            yaw_error += 360

        yaw_error = max(-50, min(50, yaw_error))  # Clamp yaw error

        kp = self.dynamic_kp_adjustment(current_state)
        adjustment = kp * yaw_error

        if direction == self.DIRECTION_MAP["FORWARD"]:
            left_pwm = self.base_speed + adjustment
            right_pwm = self.base_speed - adjustment
        elif direction == self.DIRECTION_MAP["BACKWARD"]:
            left_pwm = self.base_speed - adjustment
            right_pwm = self.base_speed + adjustment
        else:
            left_pwm = self.base_speed
            right_pwm = self.base_speed

        left_pwm = max(60, min(80, left_pwm))
        right_pwm = max(60, min(80, right_pwm))

        self.send_motor_command(direction, int(left_pwm), int(right_pwm))
        print(f"Motors updated -> State: {current_state}, Left PWM: {left_pwm:.2f}, Right PWM: {right_pwm:.2f}, Yaw Error: {yaw_error:.2f}")

    def check_timeout(self):
        """
        Check if the time since the last command exceeds the timeout, and stop motors if necessary.
        """
        if time.time() - self.last_command_time > self.command_timeout:
            print("Command timeout reached. Stopping motors for safety.")
            self.send_motor_command(-1, 0, 0)

    def close(self):
        """Close the serial connection to the Uno."""
        try:
            self.uno_serial.close()
            print("Serial connection to Uno closed.")
        except Exception as e:
            print(f"Error closing Uno serial connection: {e}")


if __name__ == "__main__":
    try:
        motor_control = MotorControl("/dev/ttyUSB0", base_speed=66)

        while True:
            imu_data = {"yaw": 0}  # Replace with real IMU data
            current_state = "FORWARD"  # Example state: "FORWARD"
            motor_control.update_motors(imu_data, current_state)
            time.sleep(0.1)
            motor_control.check_timeout()

    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        if 'motor_control' in locals():
            motor_control.close()
