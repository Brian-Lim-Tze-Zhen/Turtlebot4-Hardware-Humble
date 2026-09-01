#!/usr/bin/env python3
import numpy as np
import cv2
import open_clip
import torch
from PIL import Image
from ultralytics import YOLO

PROMPTS = [
    "two people facing each other",
    "two people not facing each other",
]
VLM_MIN_CONFIDENCE = 0.55
KPT_MIN_CONF = 0.30  # below this, a pose keypoint's coordinate is untrusted


class FacingClassifier:
    def __init__(self, pose_model_path="yolov8n-pose.pt"):
        print("Loading MobileCLIP-S1...")
        self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
            "MobileCLIP-S1", pretrained="datacompdr"
        )
        self.clip_model.eval()
        tokenizer = open_clip.get_tokenizer("MobileCLIP-S1")
        text_tokens = tokenizer(PROMPTS)
        with torch.no_grad():
            text_features = self.clip_model.encode_text(text_tokens)
            self.clip_text_features = text_features / text_features.norm(dim=-1, keepdim=True)

        print("Loading YOLOv8-pose...")
        self.pose_model = YOLO(pose_model_path)

    # ---- Method 1: MobileCLIP ----
    def classify_vlm(self, crop_bgr):
        crop_rgb = crop_bgr[:, :, ::-1]
        pil_image = Image.fromarray(crop_rgb)
        image_input = self.clip_preprocess(pil_image).unsqueeze(0)

        with torch.no_grad():
            image_features = self.clip_model.encode_image(image_input)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            similarities = (100.0 * image_features @ self.clip_text_features.T).softmax(dim=-1)

        best_idx = int(similarities.argmax())
        best_score = float(similarities[0, best_idx])
        print(f"  [VLM] '{PROMPTS[best_idx]}' score={best_score:.3f}")

        if best_score < VLM_MIN_CONFIDENCE:
            return None  # inconclusive
        return best_idx == 0

    # ---- Method 2: head-gaze (from test_pose.py) ----
    @staticmethod
    def _extract_head_heading(kpts):
        nose, l_eye, r_eye = kpts[0], kpts[1], kpts[2]
        l_ear, r_ear = kpts[3], kpts[4]

        if l_ear[0] > 0 and r_ear[0] == 0 and nose[0] > 0:
            return np.array([1.0, 0.0])
        if r_ear[0] > 0 and l_ear[0] == 0 and nose[0] > 0:
            return np.array([-1.0, 0.0])
        if l_ear[0] > 0 and r_ear[0] > 0 and nose[0] > 0:
            ear_mid_x = (l_ear[0] + r_ear[0]) / 2.0
            yaw_px = nose[0] - ear_mid_x
            if yaw_px > 3.0:
                return np.array([1.0, 0.0])
            elif yaw_px < -3.0:
                return np.array([-1.0, 0.0])
            return np.array([0.0, 0.0])
        return np.array([0.0, 0.0])

    # ---- Method 3: hip heading (fallback when head is out of frame) ----
    # COCO keypoint indices: 11 = left hip, 12 = right hip.
    # Same convention as ears: only one hip visible -> profile view ->
    # facing away from the occluded side. No nose reference exists at
    # hip level, so unlike head-heading there is no frontal/semi-angle
    # case here - hips only signal a clear profile turn, nothing finer.
    @staticmethod
    def _extract_hip_heading(kpts):
        l_hip, r_hip = kpts[11], kpts[12]

        if l_hip[0] > 0 and r_hip[0] == 0:
            return np.array([1.0, 0.0])   # left hip visible -> facing RIGHT
        if r_hip[0] > 0 and l_hip[0] == 0:
            return np.array([-1.0, 0.0])  # right hip visible -> facing LEFT
        return np.array([0.0, 0.0])       # both/neither visible -> no signal

    def classify_gaze(self, crop_bgr):
        results = self.pose_model(crop_bgr, verbose=False)[0]
        kpts_list = results.keypoints.xy.cpu().numpy()
        boxes_list = results.boxes.xyxy.cpu().numpy()
        conf_list = results.keypoints.conf.cpu().numpy() if results.keypoints.conf is not None else None

        if len(kpts_list) != 2:
            print(f"  [Gaze] expected 2 people in crop, found {len(kpts_list)} -> inconclusive")
            return None

        people = []
        for idx, (kpts, box) in enumerate(zip(kpts_list, boxes_list)):
            cx = (box[0] + box[2]) / 2.0

            # A keypoint's x is only trustworthy above KPT_MIN_CONF - below
            # that, YOLOv8-pose still emits a placeholder coordinate (often
            # x!=0, y=0) for a keypoint it never actually detected, e.g. a
            # head that's out of frame. x>0 alone can't tell a real
            # detection from that placeholder; confidence can. Zero out
            # low-confidence points so the heading extractors see them as
            # "not visible", same as a true (0,0).
            kpts_gated = kpts.copy()
            if conf_list is not None:
                low_conf = conf_list[idx] < KPT_MIN_CONF
                kpts_gated[low_conf] = 0.0

            head_vec = self._extract_head_heading(kpts_gated)
            source = "head"

            # Head gave no signal (out of frame or looking dead-on) ->
            # try hips as a lower-body fallback for the same person.
            if head_vec[0] == 0.0:
                hip_vec = self._extract_hip_heading(kpts_gated)
                if hip_vec[0] != 0.0:
                    head_vec = hip_vec
                    source = "hip"

            people.append({"cx": cx, "head_vec": head_vec, "source": source})

        left_p, right_p = sorted(people, key=lambda p: p["cx"])
        facing = (left_p["head_vec"][0] > 0.5) and (right_p["head_vec"][0] < -0.5)

        if left_p["head_vec"][0] == 0.0 and right_p["head_vec"][0] == 0.0:
            print("  [Gaze] no head or hip keypoints detected -> inconclusive")
            return None

        print(
            f"  [Gaze] left={left_p['source']}({left_p['head_vec'][0]:.0f}) "
            f"right={right_p['source']}({right_p['head_vec'][0]:.0f}) "
            f"facing={facing}"
        )
        return facing

    # ---- Combined: VLM first, head-gaze fallback ----
    def classify(self, crop_bgr):
        print("Running VLM...")
        vlm_result = self.classify_vlm(crop_bgr)
        if vlm_result is not None:
            print(f"VLM confident -> using VLM result: {vlm_result}")
            return vlm_result

        print("VLM inconclusive, falling back to head-gaze...")
        gaze_result = self.classify_gaze(crop_bgr)
        if gaze_result is not None:
            print(f"Gaze confident -> using gaze result: {gaze_result}")
            return gaze_result

        print("Both methods inconclusive -> no verdict")
        return None


if __name__ == "__main__":
    import sys
    img_path = sys.argv[1] if len(sys.argv) > 1 else "fullbody.jpg"
    frame = cv2.imread(img_path)
    if frame is None:
        print(f"Could not read {img_path}")
        sys.exit(1)

    clf = FacingClassifier("yolov8n-pose.pt")
    print(f"\n--- Classifying {img_path} ---")
    result = clf.classify(frame)
    print(f"\nFinal result: {result}")
