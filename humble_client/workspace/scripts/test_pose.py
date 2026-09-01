#!/usr/bin/env python3
import math
import itertools
import cv2
import numpy as np
from ultralytics import YOLO

class SocialGroupAnalyzer:
    def __init__(self, model_path="yolov8n-pose.pt"):
        self.pose_model = YOLO(model_path)

    def extract_head_heading(self, kpts):
        """
        Estimates a 2D horizontal unit vector (hx, 0) for the head gaze
        in the image plane (+1.0 = looking right, -1.0 = looking left).
        """
        nose = kpts[0]
        l_eye, r_eye = kpts[1], kpts[2]
        l_ear, r_ear = kpts[3], kpts[4]

        # Profile View: only one ear visible
        if l_ear[0] > 0 and r_ear[0] == 0 and nose[0] > 0:
            return np.array([1.0, 0.0])  # Looking RIGHT
        if r_ear[0] > 0 and l_ear[0] == 0 and nose[0] > 0:
            return np.array([-1.0, 0.0])  # Looking LEFT

        # Frontal / Semi-Angled View: both ears visible
        if l_ear[0] > 0 and r_ear[0] > 0 and nose[0] > 0:
            ear_mid_x = (l_ear[0] + r_ear[0]) / 2.0
            yaw_px = nose[0] - ear_mid_x
            if yaw_px > 3.0:
                return np.array([1.0, 0.0])   # Looking RIGHT
            elif yaw_px < -3.0:
                return np.array([-1.0, 0.0])  # Looking LEFT
            else:
                return np.array([0.0, 0.0])   # Neutral / Forward

        return np.array([0.0, 0.0])

    def evaluate_dyad(self, p1, p2):
        """Evaluates a 2-person candidate pair."""
        # Horizontal ordering in image plane
        left_p, right_p = (p1, p2) if p1["center"][0] < p2["center"][0] else (p2, p1)

        # Left person must look RIGHT (+X), Right person must look LEFT (-X)
        facing = (left_p["head_vec"][0] > 0.5) and (right_p["head_vec"][0] < -0.5)

        dx = p2["center"][0] - p1["center"][0]
        dy = p2["center"][1] - p1["center"][1]
        dist = math.hypot(dx, dy)

        return {
            "type": "dyad_conversation" if facing else "dyad_not_conversing",
            "members": [p1["id"], p2["id"]],
            "center": ((p1["center"][0] + p2["center"][0]) / 2.0, (p1["center"][1] + p2["center"][1]) / 2.0),
            "is_valid": facing,
            "span_px": dist
        }

    def evaluate_triad(self, p1, p2, p3):
        """Evaluates a 3-person candidate triad."""
        pts = np.array([p1["center"], p2["center"], p3["center"]])
        vecs = np.array([p1["head_vec"], p2["head_vec"], p3["head_vec"]])

        # 1. Collinearity Check via Triangle Area
        area = 0.5 * abs(
            pts[0, 0] * (pts[1, 1] - pts[2, 1]) +
            pts[1, 0] * (pts[2, 1] - pts[0, 1]) +
            pts[2, 0] * (pts[0, 1] - pts[1, 1])
        )
        if area < 500.0:  # Pixel area threshold
            return {
                "type": "triad_queue_or_collinear",
                "members": [p1["id"], p2["id"], p3["id"]],
                "is_valid": False,
                "reason": "Collinear arrangement"
            }

        # 2. Centroid (Candidate O-Space center)
        centroid = np.mean(pts, axis=0)

        # 3. Gaze convergence towards centroid
        inward_count = 0
        alignments = []
        for i in range(3):
            to_center = centroid - pts[i]
            norm = np.linalg.norm(to_center)
            if norm < 1e-4:
                continue
            to_center_unit = to_center / norm
            # Project 2D head gaze vector onto center direction vector
            align = np.dot(vecs[i], to_center_unit)
            alignments.append(align)
            if align > 0.3:  # Facing general direction of center
                inward_count += 1

        is_group = (inward_count >= 2)
        return {
            "type": "triad_conversation" if is_group else "triad_not_conversing",
            "members": [p1["id"], p2["id"], p3["id"]],
            "center": (float(centroid[0]), float(centroid[1])),
            "is_valid": is_group,
            "inward_count": inward_count,
            "alignments": [round(float(a), 2) for a in alignments]
        }

    def analyze_frame(self, frame_or_path):
        if isinstance(frame_or_path, str):
            frame = cv2.imread(frame_or_path)
        else:
            frame = frame_or_path

        results = self.pose_model(frame, verbose=False)[0]
        kpts_list = results.keypoints.xy.cpu().numpy()
        boxes_list = results.boxes.xyxy.cpu().numpy()

        people = []
        for i, (kpts, box) in enumerate(zip(kpts_list, boxes_list)):
            cx = (box[0] + box[2]) / 2.0
            cy = (box[1] + box[3]) / 2.0
            head_vec = self.extract_head_heading(kpts)

            people.append({
                "id": i,
                "box": box,
                "center": (cx, cy),
                "head_vec": head_vec
            })

        n_people = len(people)
        print(f"Detected {n_people} person(s) in scene.")

        if n_people == 2:
            verdict = self.evaluate_dyad(people[0], people[1])
            return [verdict]

        elif n_people == 3:
            verdict = self.evaluate_triad(people[0], people[1], people[2])
            return [verdict]

        elif n_people > 3:
            # Evaluate all triads and dyads
            detected_groups = []
            for combo in itertools.combinations(people, 3):
                res = self.evaluate_triad(*combo)
                if res["is_valid"]:
                    detected_groups.append(res)
            return detected_groups

        return []


if __name__ == "__main__":
    analyzer = SocialGroupAnalyzer("yolov8n-pose.pt")
    results = analyzer.analyze_frame("legs.jpg")

    print("\n--- Social Formation Output ---")
    for res in results:
        print(res)