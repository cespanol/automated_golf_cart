class CommandProcessor:
    def __init__(self, motor_control, imu_processor):
        self.motor_control = motor_control
        self.imu_processor = imu_processor
        self.state = "STOP"

    def process_command(self, command_data):
        command = command_data.get("button", "")
        imu_data = self.imu_processor.get_latest_data()
        
        if command == "w":
            self.transition_to("FORWARD", imu_data)
        elif command == "s":
            self.transition_to("BACKWARD", imu_data)
        elif command == "a":
            self.transition_to("LEFT", imu_data)
        elif command == "d":
            self.transition_to("RIGHT", imu_data)
        else:
            self.transition_to("STOP", imu_data)

    def transition_to(self, new_state, imu_data=None):
        if self.state != new_state:
            print(f"Transitioning from {self.state} to {new_state}")
            self.state = new_state
            
            # Reset the baseline yaw for all transitions
            if imu_data:
                self.motor_control.set_baseline_yaw(imu_data.get("yaw", 0))

    def get_state(self):
        return self.state
