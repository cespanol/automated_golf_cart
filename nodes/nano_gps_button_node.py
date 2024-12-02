import asyncio
import json
import paho.mqtt.client as mqtt
import serial
from bleak import BleakScanner, BleakClient

# MQTT Broker settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
BUTTON_TOPIC = "golfcart/buttons"
GPS_TOPIC = "golfcart/gps"
READY_TOPIC = "golfcart/buttons/ready"

# BLE settings
NANO_ADDRESS = "D4:D4:DA:4F:7F:82"
BUTTON_CHARACTERISTIC_UUID = "12345678-1234-5678-1234-56789abcdef1"

# GPS module settings
GPS_PORT = "/dev/ttyUSB0"  # Adjust this to match the GPS module's UART port
GPS_BAUDRATE = 38400       # Baud rate for the BE-880Q module

# Initialize MQTT client
mqtt_client = mqtt.Client()

# GPS Reader
async def read_gps_data():
    """Read GPS data from the module and publish it to MQTT."""
    try:
        with serial.Serial(GPS_PORT, GPS_BAUDRATE, timeout=1) as gps_serial:
            print("Connected to GPS module.")
            buffer = ""
            while True:
                byte_data = gps_serial.read_until(b'\n')  # Read until newline
                try:
                    line = byte_data.decode('utf-8', errors='ignore').strip()
                    if line.startswith("$"):  # Check if it's a valid NMEA sentence
                        print(f"Received NMEA sentence: {line}")
                        # Publish the raw NMEA sentence to the MQTT topic
                        mqtt_message = json.dumps({"nmea": line})
                        mqtt_client.publish(GPS_TOPIC, mqtt_message)
                        print(f"Published GPS Data: {mqtt_message}")
                except Exception as e:
                    print(f"Error decoding GPS data: {e}")
                await asyncio.sleep(0.1)  # Prevent high CPU usage
    except Exception as e:
        print(f"Error initializing GPS module: {e}")



# BLE notification handler for button presses
def ble_notification_handler(sender, data):
    """Handle BLE notifications from the Nano."""
    button = data.decode("utf-8")
    print(f"Button Pressed: {button}")

    # Publish button press to MQTT
    mqtt_message = json.dumps({"button": button})
    mqtt_client.publish(BUTTON_TOPIC, mqtt_message)
    print(f"Published Button Command: {mqtt_message}")


async def connect_to_nano():
    """Handle Nano BLE connection with retries."""
    while True:
        print("Scanning for Nano BLE device...")
        devices = await BleakScanner.discover()
        nano_device = next((d for d in devices if d.address.lower() == NANO_ADDRESS.lower()), None)

        if not nano_device:
            print("Nano device not found. Retrying in 5 seconds...")
            await asyncio.sleep(5)
            continue

        print(f"Connecting to Nano ({nano_device.address})...")
        try:
            async with BleakClient(nano_device.address, timeout=30.0) as client:
                print("Connected to Nano.")
                mqtt_client.publish(READY_TOPIC, json.dumps({"status": "ready"}))

                await client.start_notify(BUTTON_CHARACTERISTIC_UUID, ble_notification_handler)
                print("Listening for button presses...")

                while True:
                    if not client.is_connected:
                        raise Exception("Disconnected from Nano BLE device")
                    await asyncio.sleep(0.1)  # Keep connection alive

        except Exception as e:
            print(f"Error or disconnection: {e}. Retrying in 5 seconds...")
            await asyncio.sleep(5)


async def main():
    # MQTT setup
    mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)
    mqtt_client.loop_start()

    try:
        # Run BLE and GPS tasks concurrently
        await asyncio.gather(connect_to_nano(), read_gps_data())
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
