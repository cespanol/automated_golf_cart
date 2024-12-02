import asyncio
import json
from bleak import BleakClient, BleakScanner
import paho.mqtt.client as mqtt

# MQTT settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPIC_IMU_DATA = "golfcart/imu"
READY_TOPIC = "golfcart/imu/ready"

# WT901 BLE settings
WT901_ADDRESS = "DA:10:A2:CE:17:23"
WT901_CHARACTERISTIC_UUID = "0000ffe4-0000-1000-8000-00805f9a34fb"

# Calibration offsets
accel_offsets = {'ax': -31.99, 'ay': -32.00, 'az': -0.01}
gyro_offsets = {'wx': 0.0, 'wy': 0.0, 'wz': 0.0}
angle_offsets = {'roll': -359.72, 'pitch': -0.60, 'yaw': -31.72}

def parse_imu_data(data):
    """Parse raw data from WT901 into calibrated IMU readings."""
    if len(data) < 20:
        print("Incomplete data received!")
        return None

    ax = ((data[3] << 8 | data[2]) / 32768.0 * 16) + accel_offsets['ax']
    ay = ((data[5] << 8 | data[4]) / 32768.0 * 16) + accel_offsets['ay']
    az = ((data[7] << 8 | data[6]) / 32768.0 * 16) + accel_offsets['az']
    wx = ((data[9] << 8 | data[8]) / 32768.0 * 2000) + gyro_offsets['wx']
    wy = ((data[11] << 8 | data[10]) / 32768.0 * 2000) + gyro_offsets['wy']
    wz = ((data[13] << 8 | data[12]) / 32768.0 * 2000) + gyro_offsets['wz']
    roll = ((data[15] << 8 | data[14]) / 32768.0 * 180) + angle_offsets['roll']
    pitch = ((data[17] << 8 | data[16]) / 32768.0 * 180) + angle_offsets['pitch']
    yaw = ((data[19] << 8 | data[18]) / 32768.0 * 180) + angle_offsets['yaw']

    return {
        "ax": ax, "ay": ay, "az": az,
        "wx": wx, "wy": wy, "wz": wz,
        "roll": roll, "pitch": pitch, "yaw": yaw
    }


async def connect_to_wt901(mqtt_client):
    """Handle WT901 BLE connection with retries."""
    while True:
        print("Scanning for WT901 BLE device...")
        devices = await BleakScanner.discover()
        wt901_device = next((d for d in devices if d.address.lower() == WT901_ADDRESS.lower()), None)

        if not wt901_device:
            print("WT901 device not found. Retrying in 5 seconds...")
            await asyncio.sleep(5)
            continue

        print(f"Connecting to WT901 ({wt901_device.address})...")
        try:
            async with BleakClient(wt901_device.address, timeout=30.0) as client:
                print("Connected to WT901.")
                mqtt_client.publish(READY_TOPIC, json.dumps({"status": "ready"}))

                def notification_handler(sender, data):
                    imu_data = parse_imu_data(data)
                    if imu_data:
                        mqtt_message = json.dumps(imu_data)
                        mqtt_client.publish(TOPIC_IMU_DATA, mqtt_message)
                        print(f"Published IMU Data: {mqtt_message}")

                await client.start_notify(WT901_CHARACTERISTIC_UUID, notification_handler)
                print("Listening for IMU data...")

                while True:
                    if not client.is_connected:
                        raise Exception("Disconnected from WT901 BLE device")
                    await asyncio.sleep(0.1)  # Keep connection alive

        except Exception as e:
            print(f"Error with WT901 connection: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)


async def main():
    # MQTT Client setup
    mqtt_client = mqtt.Client()
    mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
    mqtt_client.loop_start()

    try:
        await connect_to_wt901(mqtt_client)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
