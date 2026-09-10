import sys, cv2
from ultralytics import YOLO

img_path = sys.argv[1]
model = YOLO("yolov8n-pose.pt")
frame = cv2.imread(img_path)
results = model(frame, verbose=False)[0]
boxes = results.boxes.xyxy.cpu().numpy()
confs = results.boxes.conf.cpu().numpy()
print(f"Detections: {len(boxes)}")
for i, (box, conf) in enumerate(zip(boxes, confs)):
    print(f"  person {i}: box={box.round(1)} conf={conf:.3f}")
