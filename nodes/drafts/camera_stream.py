import cv2
import numpy as np
from paho.mqtt import client as mqtt_client

# Connect to ESP32 Camera Stream
camera_url = "http://192.168.4.2/stream"
cap = cv2.VideoCapture(camera_url)

# MQTT setup
mqtt_broker = 'localhost'
mqtt_port = 1883
mqtt_topic = "golfcart/detections"
client = mqtt_client.Client("RaspberryPiCameraNode")
client.connect(mqtt_broker, mqtt_port)

def publish_detection(detection):
    client.publish(mqtt_topic, detection)

# Load object detection model
net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet.caffemodel')

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Perform object detection
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    # Iterate over detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([frame.shape[1], frame.shape[0], frame.shape[1], frame.shape[0]])
            (startX, startY, endX, endY) = box.astype("int")
            label = f"Object {idx}: {confidence * 100:.2f}%"
            cv2.rectangle(frame, (startX, startY), (endX, endY), (255, 0, 0), 2)
            cv2.putText(frame, label, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Publish detection to MQTT
            detection_message = f"Detected object {idx} with confidence {confidence:.2f}"
            publish_detection(detection_message)

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
