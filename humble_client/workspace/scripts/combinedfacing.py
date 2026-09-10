#!/usr/bin/env python3
import numpy as np
import cv2
import open_clip
import torch
from PIL import Image
from ultralytics import YOLO

PROMPTS = [
    "people facing each other",
    "people not facing each other",
]
VLM_MIN_CONFIDENCE = 0.55
KPT_MIN_CONF = 0.30  # below this, a pose keypoint's coordinate is untrusted


class FacingClassifier:
    def __init__(self, pose_model_path="yolov8n-pose.pt"):
        self.clip_model = None  # lazy-loaded on first VLM fallback use
        self.clip_preprocess = None
        self.clip_text_features = None

        print("Loading YOLOv8-pose...")
        self.pose_model = YOLO(pose_model_path)

    def _ensure_vlm_loaded(self):
        if self.clip_model is not None:
            return
        print("Loading MobileCLIP-S1 (fallback triggered)...")
        self.clip_model, _, self.clip_preprocess = open_clip.create_model_and_transforms(
            "MobileCLIP-S1", pretrained="datacompdr"
        )
        self.clip_model.eval()
        tokenizer = open_clip.get_tokenizer("MobileCLIP-S1")
        text_tokens = tokenizer(PROMPTS)
        with torch.no_grad():
            text_features = self.clip_model.encode_text(text_tokens)
            self.clip_text_features = text_features / text_features.norm(dim=-1, keepdim=True)

    # ---- Method 1: MobileCLIP ----
    def classify_vlm(self, crop_bgr):
        self._ensure_vlm_loaded()
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
            return np.array([-1.0, 0.0])  # flipped (see note below)
        if r_ear[0] > 0 and l_ear[0] == 0 and nose[0] > 0:
            return np.array([1.0, 0.0])   # flipped (see note below)
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

    # A keypoint counts as "region visible at all" above this looser
    # threshold, even if it's below KPT_MIN_CONF (the stricter bar for
    # trusting a DIRECTION from it). Used only to distinguish "we saw
    # this person's head/hip region and it read as squared/neutral"
    # from "we saw nothing at all" (e.g. out of frame).
    KPT_PRESENCE_CONF = 0.20

    @staticmethod
    def _has_orientation_data(kpts_orig, conf_row):
        """True if nose or either ear was detected with at least
        KPT_PRESENCE_CONF - i.e. we have a real read on HEAD region
        specifically. Deliberately excludes hips: hip squareness does
        NOT prove absence of facing (a person's head can turn toward a
        conversation partner while hips stay squared - that's normal
        conversational posture), so trusting hip-only presence for the
        "confirmed not facing" conclusion produced a false negative on
        legs.jpg (hips visible, head out of frame, but the pair IS
        facing per fullbody.jpg). Only head presence is trustworthy
        enough to confirm "genuinely squared, not just no data"."""
        if conf_row is None:
            return False
        idxs = [0, 3, 4]  # nose, l_ear, r_ear
        return bool(np.any(conf_row[idxs] >= FacingClassifier.KPT_PRESENCE_CONF))

    def _person_heading(self, kpts, conf_row):
        """Confidence-gated head heading with hip fallback. Shared by
        both dyad and triad gaze logic."""
        kpts_gated = kpts.copy()
        if conf_row is not None:
            kpts_gated[conf_row < KPT_MIN_CONF] = 0.0

        head_vec = self._extract_head_heading(kpts_gated)
        source = "head"
        if head_vec[0] == 0.0:
            hip_vec = self._extract_hip_heading(kpts_gated)
            if hip_vec[0] != 0.0:
                head_vec = hip_vec
                source = "hip"
        return head_vec, source

    def _classify_gaze_dyad(self, kpts_list, boxes_list, conf_list):
        """Returns one of:
          "facing"      - classic face-to-face conversation dyad
          "side_by_side"- parallel/companion formation (shared external
                          focus, not facing each other, but still a
                          social group a robot shouldn't cut through -
                          e.g. two people standing together looking at
                          the same view, this project's talking_behind.jpeg)
          "diverging"   - headings point away from each other - genuinely
                          not a group (e.g. backtoback.jpg's posed shot)
          None          - insufficient data, unknown
        """
        people = []
        for idx, (kpts, box) in enumerate(zip(kpts_list, boxes_list)):
            cx = (box[0] + box[2]) / 2.0
            conf_row = conf_list[idx] if conf_list is not None else None
            head_vec, source = self._person_heading(kpts, conf_row)
            has_data = self._has_orientation_data(kpts, conf_row)
            people.append({"cx": cx, "head_vec": head_vec, "source": source, "has_data": has_data})

        left_p, right_p = sorted(people, key=lambda p: p["cx"])
        lx, rx = left_p["head_vec"][0], right_p["head_vec"][0]

        if lx > 0.5 and rx < -0.5:
            print(f"  [Gaze] left={left_p['source']}({lx:.0f}) right={right_p['source']}({rx:.0f}) -> facing")
            return "facing"

        if lx == 0.0 and rx == 0.0:
            if left_p["has_data"] and right_p["has_data"]:
                print("  [Gaze] both confirmed squared/no lateral turn -> side_by_side")
                return "side_by_side"
            print("  [Gaze] no head or hip keypoints detected -> inconclusive")
            self._no_signal = True
            return None

        if lx < -0.5 and rx > 0.5:
            print(f"  [Gaze] left={left_p['source']}({lx:.0f}) right={right_p['source']}({rx:.0f}) -> diverging (not a group)")
            return "diverging"

        # Any other combination (e.g. both heading the same direction,
        # or one zero/one nonzero) - conservatively treat as a companion
        # formation rather than assume no group. Under-avoiding a real
        # group is worse than over-avoiding an ambiguous one.
        print(f"  [Gaze] left={left_p['source']}({lx:.0f}) right={right_p['source']}({rx:.0f}) -> side_by_side (ambiguous, conservative)")
        return "side_by_side"

    # ---- Triad gaze: centroid convergence (ported from test_pose.py) ----
    # Only the horizontal component is available from our heading
    # vectors ((+-1, 0), never a true 2D gaze), so "convergence" here
    # means: does this person's left/right heading point toward the
    # group's horizontal centroid, not a full 2D O-space check.
    TRIAD_MIN_AREA = 500.0    # px^2; below this, treat as collinear/queue
    TRIAD_ALIGN_THRESHOLD = 0.3

    def _classify_gaze_triad(self, kpts_list, boxes_list, conf_list):
        centers = []
        headings = []
        sources = []
        for idx, (kpts, box) in enumerate(zip(kpts_list, boxes_list)):
            cx = (box[0] + box[2]) / 2.0
            cy = (box[1] + box[3]) / 2.0
            conf_row = conf_list[idx] if conf_list is not None else None
            head_vec, source = self._person_heading(kpts, conf_row)
            centers.append((cx, cy))
            headings.append(head_vec)
            sources.append(source)

        pts = np.array(centers)
        vecs = np.array(headings)

        # Collinearity check (queue-like arrangement, not a huddle)
        area = 0.5 * abs(
            pts[0, 0] * (pts[1, 1] - pts[2, 1]) +
            pts[1, 0] * (pts[2, 1] - pts[0, 1]) +
            pts[2, 0] * (pts[0, 1] - pts[1, 1])
        )
        if area < self.TRIAD_MIN_AREA:
            print(f"  [Gaze] triad collinear (area={area:.0f}) -> not a huddle")
            return "diverging"

        centroid = np.mean(pts, axis=0)
        inward_count = 0
        usable = 0
        for i in range(3):
            if headings[i][0] == 0.0:
                continue  # no signal for this person, skip rather than guess
            usable += 1
            to_center = centroid - pts[i]
            norm = np.linalg.norm(to_center)
            if norm < 1e-4:
                continue
            to_center_unit = to_center / norm
            align = float(np.dot(vecs[i], to_center_unit))
            if align > self.TRIAD_ALIGN_THRESHOLD:
                inward_count += 1

        print(
            f"  [Gaze] triad sources={sources} usable={usable}/3 "
            f"inward={inward_count}"
        )

        if usable < 2:
            print("  [Gaze] fewer than 2 people with usable heading -> inconclusive")
            if usable == 0:
                self._no_signal = True
            return None

        # Below 2 inward: not a facing huddle. Conservatively still call
        # it side_by_side (a group) rather than diverging (no group),
        # same reasoning as the dyad case - under-avoiding a real group
        # costs more than over-avoiding an ambiguous one.
        return "facing" if inward_count >= 2 else "side_by_side"

    @staticmethod
    def _iou(box_a, box_b):
        xa1, ya1 = max(box_a[0], box_b[0]), max(box_a[1], box_b[1])
        xa2, ya2 = min(box_a[2], box_b[2]), min(box_a[3], box_b[3])
        inter_w, inter_h = max(0.0, xa2 - xa1), max(0.0, ya2 - ya1)
        inter = inter_w * inter_h
        area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
        union = area_a + area_b - inter
        return inter / union if union > 0 else 0.0

    DEDUP_IOU_THRESHOLD = 0.45

    def _deduplicate(self, kpts_list, boxes_list, conf_list, box_confs):
        """Drop overlapping duplicate person detections, keeping the
        higher-confidence box. YOLO's own NMS can still leave duplicates
        on irregular poses (e.g. crossed arms creating two plausible
        boxes for the same person) - confirmed on backtoback.jpg, which
        produced 3 boxes for 2 real people, the extra one at lower
        (but not obviously-low) confidence and heavily overlapping an
        already-kept box."""
        order = sorted(range(len(boxes_list)), key=lambda i: -box_confs[i])
        keep = []
        for i in order:
            if any(self._iou(boxes_list[i], boxes_list[j]) > self.DEDUP_IOU_THRESHOLD for j in keep):
                continue
            keep.append(i)
        keep.sort()
        if len(keep) != len(boxes_list):
            print(f"  [Gaze] deduplicated {len(boxes_list)} -> {len(keep)} detections")
        return (
            [kpts_list[i] for i in keep],
            [boxes_list[i] for i in keep],
            [conf_list[i] for i in keep] if conf_list is not None else None,
        )

    def classify_gaze(self, crop_bgr):
        self._no_signal = False  # set True if no head/hip data on anyone at all
        results = self.pose_model(crop_bgr, verbose=False)[0]
        kpts_list = results.keypoints.xy.cpu().numpy()
        boxes_list = results.boxes.xyxy.cpu().numpy()
        box_confs = results.boxes.conf.cpu().numpy()
        conf_list = results.keypoints.conf.cpu().numpy() if results.keypoints.conf is not None else None

        kpts_list, boxes_list, conf_list = self._deduplicate(kpts_list, boxes_list, conf_list, box_confs)

        n = len(kpts_list)
        if n == 2:
            return self._classify_gaze_dyad(kpts_list, boxes_list, conf_list)
        elif n == 3:
            return self._classify_gaze_triad(kpts_list, boxes_list, conf_list)
        else:
            print(f"  [Gaze] {n} people detected, only 2 or 3 supported -> inconclusive")
            return None

    # ---- Combined: gaze-first, VLM only as fallback ----
    # Empirically (6 test images), gaze was correct every time it had a
    # signal, and every VLM/gaze disagreement was VLM being wrong (its
    # documented side-on blind spot). VLM never once resolved a case
    # gaze couldn't - and on legs-only crops (no head, no hip data on
    # ANYONE), VLM was shown to guess CONFIDENTLY wrong (cropleg.jpg:
    # 0.948 for a walking, non-facing pair). That's not a gap VLM fills;
    # it's the same missing information, guessed on. So that specific
    # case (self._no_signal) skips VLM entirely and stays honestly
    # unresolved, matching what group_formation_detector.py already
    # does in production (skip and retry next cycle on bad framing).
    #
    # Return value is a group-type label, not a bool - navigation cares
    # about is_group(), not the sub-type, but the sub-type is kept for
    # logging/downstream costmap shaping (a facing pair vs a side-by-side
    # pair may want a differently-shaped avoidance zone).
    def classify(self, crop_bgr):
        print("Running gaze...")
        gaze_result = self.classify_gaze(crop_bgr)
        print(f"  Gaze result: {gaze_result}")

        if gaze_result is not None:
            print(f"Using gaze result: {gaze_result}")
            return gaze_result

        if self._no_signal:
            print("No head/hip data on anyone -> VLM would be guessing on the same blind spot, skipping")
            return None

        print("Gaze inconclusive, falling back to VLM...")
        vlm_result = self.classify_vlm(crop_bgr)
        # VLM is binary (facing / not facing) - it can't distinguish
        # side_by_side from diverging, so its "not facing" is mapped to
        # the conservative "still a group" label, not "no group".
        label = "facing" if vlm_result is True else ("side_by_side" if vlm_result is False else None)
        print(f"  VLM result: {vlm_result} -> {label}")

        if label is not None:
            print(f"Using VLM result: {label}")
            return label

        print("Both methods inconclusive -> no verdict")
        return None

    @staticmethod
    def is_group(label):
        """The one question navigation actually needs answered: does
        this pair/triad occupy a social space the robot should avoid
        cutting through? True for "facing" and "side_by_side"; False
        for "diverging" (genuinely not a group) and unknown (None)."""
        return label in ("facing", "side_by_side")


if __name__ == "__main__":
    import sys
    img_path = sys.argv[1] if len(sys.argv) > 1 else "two.jpg"
    frame = cv2.imread(img_path)
    if frame is None:
        print(f"Could not read {img_path}")
        sys.exit(1)

    clf = FacingClassifier("yolov8n-pose.pt")
    print(f"\n--- Classifying {img_path} ---")
    result = clf.classify(frame)
    print(f"\nFinal label: {result}")
    print(f"Is a group (robot should avoid): {FacingClassifier.is_group(result)}")