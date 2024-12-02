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
CONFIG_PATH = "ssd_mobilenet_v3_large_coco_2020_01_14/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"

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

    # publish timestamp for syncronization with lidar
    # After detections are processed

    # detections_packet = {
    #     "timestamp": time.time(),
    #     "detections": [
    #         {
    #             "label": det["label"],
    #             "bbox": det["box"],  # [x, y, w, h]
    #             "confidence": det["confidence"]
    #         }
    #         for det in detections
    #     ]
    # }
    # mqtt_client.publish(TOPIC_CAMERA_DETECTIONS, json.dumps(detections_packet))


def publish_motion_command(horizontal_offset, distance):
    command = {
        "steering": horizontal_offset,
        "speed": distance
    }
    mqtt_client.publish("golfcart/motion", json.dumps(command))
    print(f"Published motion command: {command}")

def calculate_motion_command(box, frame_width, frame_height):
    x, y, w, h = box
    center_x = x + w / 2
    center_y = y + h / 2

    # Horizontal offset (steering adjustment)
    horizontal_offset = (center_x - frame_width / 2) / (frame_width / 2)

    # Distance estimation based on box size (speed adjustment)
    distance = frame_height / h  # Assuming a larger height means closer

    return horizontal_offset, distance


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

        result = net.detect(frame, confThreshold=0.5, nmsThreshold=0.4)

        # Check if there are any detections
        if result and len(result) == 3:
            classes, confidences, boxes = result

            # Check if classes, confidences, and boxes are non-empty
            if len(classes) > 0 and len(confidences) > 0 and len(boxes) > 0:
                # Convert class IDs to their corresponding labels
                labeled_classes = [COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else "unknown" for class_id in classes.flatten()]
                
                print(f"Detected classes: {labeled_classes}")
                print(f"Confidences: {confidences}")
                print(f"Boxes: {boxes}")

                detections = [
                    {
                        "label": COCO_CLASSES[class_id] if 0 <= class_id < len(COCO_CLASSES) else "unknown",
                        "box": box,
                        "confidence": confidence
                    }
                    for class_id, confidence, box in zip(classes.flatten(), confidences.flatten(), boxes)
                ]
                target_detections = [det for det in detections if det["label"] == "person"]

                for detection in target_detections:
                    horizontal_offset, distance = calculate_motion_command(
                        detection["box"], frame.shape[1], frame.shape[0]
                    )
                    print(f"Bounding box: {detection['box']}, Steering: {horizontal_offset}, Speed: {distance}")
                    publish_motion_command(horizontal_offset, distance)

            else:
                print("No valid detections.")
        else:
            print("No detections available.")

        # Optional: Save processed frame and display
        frame_path = os.path.join(OUTPUT_DIR, f"frame_{frame_count:04d}.jpg")
        cv2.imwrite(frame_path, frame)
        print(f"Frame saved: {frame_path} \n")
        frame_count += 1

    cap.release()




if __name__ == "__main__":
    main()
