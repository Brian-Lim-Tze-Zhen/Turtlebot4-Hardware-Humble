#!/usr/bin/env python3
import sys
sys.path.insert(0, "/root/thesis_social_navigation_ws/scripts")
import cv2
from social_group_detector_node import FacingClassifier

clf = FacingClassifier("/root/thesis_social_navigation_ws/scripts/yolov8n-pose.pt")
cap = cv2.VideoCapture(0)
label = "..."
n = 0
while True:
    ok, frame = cap.read()
    if not ok:
        break
    n += 1
    if n % 5 == 0:
        label = str(clf.classify(frame, debug_log=lambda s: print("[classify]", s)))
    cv2.putText(frame, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.imshow("facing test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()