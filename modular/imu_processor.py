class IMUProcessor:
    def __init__(self):
        self.latest_data = {}  # Holds the filtered IMU data
        self.previous_yaw = 0  # For filtering yaw values
        self.alpha = 0.1  # Low-pass filter coefficient (adjust as needed)

    def process_imu_data(self, data):
        """
        Processes incoming IMU data, applying a low-pass filter to the yaw.
        Args:
            data (dict): Raw IMU data containing yaw information.
        """
        raw_yaw = data.get("yaw", 0)
        filtered_yaw = self.alpha * raw_yaw + (1 - self.alpha) * self.previous_yaw
        self.previous_yaw = filtered_yaw
        self.latest_data = {"yaw": filtered_yaw}

    def get_latest_data(self):
        """
        Returns the latest filtered IMU data.
        Returns:
            dict: Processed IMU data containing filtered yaw.
        """
        return self.latest_data
