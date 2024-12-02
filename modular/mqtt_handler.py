import paho.mqtt.client as mqtt
import json
import time

class MQTTHandler:
    def __init__(self, broker_address, broker_port, topics):
        self.client = mqtt.Client()
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.topics = topics
        self.callbacks = {}

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def set_callback(self, topic, callback):
        self.callbacks[topic] = callback

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected to MQTT Broker with result code {rc}")
        for topic, qos in self.topics:
            self.client.subscribe((topic, qos))

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8")
        if topic in self.callbacks:
            try:
                data = json.loads(payload)
                self.callbacks[topic](data)
            except json.JSONDecodeError:
                print(f"Invalid JSON on topic {topic}: {payload}")

    def start(self):
        self.client.connect(self.broker_address, self.broker_port, 60)
        self.client.loop_start()
        while not self.client.is_connected():
            time.sleep(1)
            print("Attempting to reconnect...")


    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()
