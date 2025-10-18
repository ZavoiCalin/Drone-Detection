from ultralytics import YOLO

# Load your trained YOLOv3 model
model = YOLO("yolov3-spp-ultralytics.pt")

# Run inference on an image
results = model.predict(source="drone_1.jpg", conf=0.25, show=True, save=True)

# Access detections (optional)
for box in results[0].boxes:
    print(f"Coordinates: {box.xyxy}, Confidence: {box.conf}, Class: {box.cls}")
