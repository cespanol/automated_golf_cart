import cv2
import paho.mqtt.client as mqtt
import json
import os

COCO_CLASSES = [
    "background", "person", "bicycle", "car", "motorcycle", "airplane", "bus", 
    "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", 
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", 
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", 
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", 
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", 
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", 
    "toilet", "TV", "laptop", "mouse", "remote", "keyboard", "cell phone", 
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", 
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]


# MQTT Broker settings
BROKER_ADDRESS = "localhost"
BROKER_PORT = 1883
TOPIC = "golfcart/camera/detections"

# ESP32 Camera Stream URL
# CAMERA_URL = "http://192.168.1.204:81/stream"
CAMERA_URL = "http://192.168.42.150:81/stream"


# Load pre-trained MobileNet SSD model and config
MODEL_PATH = "ssd_mobilenet_v3_large_coco_2020_01_14/frozen_inference_graph.pb"
CONFIG_PATH = "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"

# Directory to save processed frames (for debugging)
OUTPUT_DIR = "output_frames"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Initialize MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(BROKER_ADDRESS, BROKER_PORT, 60)

def publish_detection(detections):
    """Publish detections to MQTT."""
    message = json.dumps(detections)
    mqtt_client.publish(TOPIC, message)
    print(f"Published: {message}")

def main():
    # Load the model
    net = cv2.dnn_DetectionModel(MODEL_PATH, CONFIG_PATH)
    net.setInputSize(320, 320)
    net.setInputScale(1.0 / 127.5)
    net.setInputMean((127.5, 127.5, 127.5))
    net.setInputSwapRB(True)

    # Open the video stream
    cap = cv2.VideoCapture(CAMERA_URL, cv2.CAP_FFMPEG)

    if not cap.isOpened():
        print("Failed to open camera stream.")
        return

    print("Connected to camera stream.")
    frame_count = 0  # Frame counter for saving files

    # Object detection loop
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame.")
            break

        # Perform object detection
        result = net.detect(frame, confThreshold=0.5, nmsThreshold=0.4)

        # Ensure detection results are valid
        if len(result) == 3:
            classes, confidences, boxes = result
            detections = []
            for class_id, confidence, box in zip(classes, confidences, boxes):
                x, y, w, h = box


                # Draw bounding boxes and labels on the frame
                label = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else "unknown"

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detections.append({
                    "class_id": int(class_id),
                    "label": label,
                    "confidence": float(confidence),
                    "box": [int(x), int(y), int(w), int(h)]
                })


            # Publish detections
            

            publish_detection(detections)

        # Save the processed frame to disk
        frame_path = os.path.join(OUTPUT_DIR, f"frame_{frame_count:04d}.jpg")
        cv2.imwrite(frame_path, frame)
        print(f"Frame saved: {frame_path}")

        frame_count += 1

    cap.release()

if __name__ == "__main__":
    main()
