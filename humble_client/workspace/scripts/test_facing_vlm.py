#!/usr/bin/env python3
"""
test_facing_vlm.py

Offline test: would the VLM (MobileCLIP) add anything to the facing
classifier? Runs OUTSIDE the detector node and changes nothing in it.

WHY
  In social_group_detector_node_lidarhold.py the VLM fallback is never
  reached: classify() only calls classify_vlm() when the keypoint rule
  returns None with _no_signal False, and every None path sets
  _no_signal True. So the deployed facing decision is the keypoint rule
  alone. Before wiring the VLM in, measure it on the crops the robot
  actually produced.

WHAT IT DOES
  For every image in --crops (default: debug_crops/, which the detector
  fills with every crop it tried to classify):
    1. keypoint rule   - the SAME FacingClassifier the node uses
    2. people count    - how many people the pose model finds in the crop
    3. VLM score       - P(facing) for each prompt set below, no threshold
  and writes
    <out>/results.csv  - one row per crop
    <out>/report.html  - every crop with both verdicts, disagreements first
  plus a summary on the terminal: label counts, keypoint-vs-VLM
  cross-table, and time per image (the VLM runs on the same CPU that
  already struggled to hold the controller at 20 Hz).

GROUND TRUTH (optional)
  Put a CSV at --labels with lines "filename,truth", truth being one of
  facing / not_facing / no_pair. Accuracy is then reported for the
  keypoint rule, each VLM prompt set, and the hybrid (keypoint rule
  first, VLM only where the rule gives no answer).

RUN (inside the container)
  python3 scripts/test_facing_vlm.py
  python3 scripts/test_facing_vlm.py --limit 60          # quick look
  python3 scripts/test_facing_vlm.py --labels debug_crops_labels.csv
"""

import argparse
import csv
import html
import os
import sys
import time
from collections import Counter

import cv2
import numpy as np
import torch
import open_clip
from PIL import Image as PILImage

WS = "/root/thesis_social_navigation_ws"
sys.path.insert(0, f"{WS}/scripts")
# Same class the node runs - not a copy, so the keypoint verdicts here
# are the node's verdicts.
from social_group_detector_node_lidarhold import (  # noqa: E402
    FacingClassifier, VLM_MIN_CONFIDENCE)

# Prompt sets to compare. "node" is what the node would use if its VLM
# path were reachable. Index 0 of each set is the FACING prompt.
PROMPT_SETS = {
    "node": ["people facing each other",
             "people not facing each other"],
    "talking": ["two people standing face to face having a conversation",
                "two people standing next to each other looking the same way"],
    "with_empty": ["two people facing each other",
                   "two people not facing each other",
                   "one person standing alone",
                   "an empty corridor with no people"],
}


def vlm_probs(clf, text_feats, crop_bgr):
    """Softmax over one prompt set, same maths as classify_vlm()."""
    pil = PILImage.fromarray(crop_bgr[:, :, ::-1])
    img = clf.clip_preprocess(pil).unsqueeze(0)
    with torch.no_grad():
        f = clf.clip_model.encode_image(img)
        f = f / f.norm(dim=-1, keepdim=True)
        return (100.0 * f @ text_feats.T).softmax(dim=-1)[0].tolist()


def count_people(clf, crop_bgr):
    """People in the crop after the node's own de-duplication."""
    r = clf.pose_model(crop_bgr, verbose=False)[0]
    if r.boxes is None or len(r.boxes) == 0:
        return 0
    k, b, _ = clf._deduplicate(
        r.keypoints.xy.cpu().numpy(), r.boxes.xyxy.cpu().numpy(),
        r.keypoints.conf.cpu().numpy() if r.keypoints.conf is not None else None,
        r.boxes.conf.cpu().numpy())
    return len(b)


def vlm_verdict(p_facing_best, best_idx, thr):
    """facing / not_facing / abstain, as classify_vlm() would decide."""
    if p_facing_best < thr:
        return "abstain"
    return "facing" if best_idx == 0 else "not_facing"


def to_truth_class(label):
    """Map a keypoint label onto the 3 ground-truth classes."""
    if label == "facing":
        return "facing"
    if label in ("side_by_side", "diverging"):
        return "not_facing"
    return "no_answer"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default=f"{WS}/debug_crops")
    ap.add_argument("--out", default=f"{WS}/debug_crops_vlm_report")
    ap.add_argument("--labels", default=None)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--threshold", type=float, default=VLM_MIN_CONFIDENCE)
    ap.add_argument("--pose", default=f"{WS}/scripts/yolov8n-pose.pt")
    args = ap.parse_args()

    files = sorted(f for f in os.listdir(args.crops)
                   if f.lower().endswith((".jpg", ".jpeg", ".png")))
    if args.limit:
        step = max(1, len(files) // args.limit)
        files = files[::step][:args.limit]     # spread over all sessions
    if not files:
        sys.exit(f"no images in {args.crops}")
    os.makedirs(args.out, exist_ok=True)

    truth = {}
    if args.labels:
        with open(args.labels) as fh:
            for row in csv.reader(fh):
                if len(row) >= 2 and not row[0].startswith("#"):
                    truth[row[0].strip()] = row[1].strip()

    clf = FacingClassifier(args.pose)
    t0 = time.time()
    clf._ensure_vlm_loaded()
    print(f"MobileCLIP loaded in {time.time() - t0:.1f} s")
    tok = open_clip.get_tokenizer("MobileCLIP-S1")
    feats = {}
    with torch.no_grad():
        for name, prompts in PROMPT_SETS.items():
            tf = clf.clip_model.encode_text(tok(prompts))
            feats[name] = tf / tf.norm(dim=-1, keepdim=True)

    rows, t_kpt, t_vlm = [], [], []
    for i, fn in enumerate(files):
        img = cv2.imread(os.path.join(args.crops, fn))
        if img is None:
            continue
        log = []
        t = time.time()
        kpt = clf.classify_gaze_dyad(img, debug_log=log.append)
        t_kpt.append(time.time() - t)
        n_people = count_people(clf, img)
        row = {"file": fn, "w": img.shape[1], "h": img.shape[0],
               "people": n_people, "kpt": kpt or "none",
               "kpt_detail": log[0] if log else "",
               "truth": truth.get(fn, "")}
        for name in PROMPT_SETS:
            t = time.time()
            p = vlm_probs(clf, feats[name], img)
            if name == "node":
                t_vlm.append(time.time() - t)
            best = int(np.argmax(p))
            row[f"vlm_{name}_pfacing"] = round(p[0], 3)
            row[f"vlm_{name}"] = vlm_verdict(p[best], best, args.threshold) \
                if best <= 1 else "no_pair"
        # Hybrid: keypoint rule first; the VLM only where the rule has no
        # answer but the pose model does see two people.
        k = to_truth_class(row["kpt"])
        if k == "no_answer" and n_people == 2 and row["vlm_node"] != "abstain":
            row["hybrid"] = row["vlm_node"]
        else:
            row["hybrid"] = k
        rows.append(row)
        if (i + 1) % 25 == 0:
            print(f"  {i + 1}/{len(files)}")

    # ---------------- csv ----------------
    cols = list(rows[0].keys())
    with open(os.path.join(args.out, "results.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)

    # ---------------- summary ----------------
    n = len(rows)
    print(f"\n{n} crops from {args.crops}")
    print("people found by pose model:",
          dict(sorted(Counter(r["people"] for r in rows).items())))
    print("keypoint rule:", dict(Counter(r["kpt"] for r in rows)))
    for name in PROMPT_SETS:
        print(f"VLM [{name}] (threshold {args.threshold}):",
              dict(Counter(r[f"vlm_{name}"] for r in rows)))
    print("\nkeypoint rule (rows) vs VLM [node] (columns):")
    vcols = ["facing", "not_facing", "abstain"]
    print(f"  {'':14s}" + "".join(f"{c:>12s}" for c in vcols))
    for kl in ["facing", "side_by_side", "diverging", "none"]:
        sub = [r for r in rows if r["kpt"] == kl]
        print(f"  {kl:14s}" + "".join(
            f"{sum(r['vlm_node'] == c for r in sub):12d}" for c in vcols))
    two = [r for r in rows if r["kpt"] == "none" and r["people"] == 2]
    print(f"\nwhere the hybrid would use the VLM (rule 'none', 2 people): "
          f"{len(two)} of {n} crops ->",
          dict(Counter(r["vlm_node"] for r in two)))
    print(f"\ntime per crop: keypoint rule {1000 * np.mean(t_kpt):.0f} ms, "
          f"VLM {1000 * np.mean(t_vlm):.0f} ms (CPU threads: "
          f"{torch.get_num_threads()})")

    if truth:
        lab = [r for r in rows if r["truth"]]
        print(f"\nACCURACY on {len(lab)} labelled crops "
              f"(truth: {dict(Counter(r['truth'] for r in lab))})")

        def score(fn_pred, title):
            pairs = [r for r in lab if r["truth"] in ("facing", "not_facing")]
            ans = [r for r in pairs if fn_pred(r) in ("facing", "not_facing")]
            ok = sum(fn_pred(r) == r["truth"] for r in ans)
            fp = sum(fn_pred(r) == "facing" and r["truth"] != "facing"
                     for r in lab)
            print(f"  {title:22s} answered {len(ans):3d}/{len(pairs):3d} real "
                  f"pairs, correct {ok:3d} "
                  f"({100 * ok / max(1, len(ans)):.0f}% of answered), "
                  f"false 'facing' on any crop: {fp}")
        score(lambda r: to_truth_class(r["kpt"]), "keypoint rule")
        for name in PROMPT_SETS:
            score(lambda r, nm=name: r[f"vlm_{nm}"], f"VLM [{name}]")
        score(lambda r: r["hybrid"], "hybrid (rule, then VLM)")

    # ---------------- html ----------------
    def rank(r):   # disagreements first, then hybrid-only cases
        k = to_truth_class(r["kpt"])
        if k in ("facing", "not_facing") and r["vlm_node"] in ("facing", "not_facing") \
                and k != r["vlm_node"]:
            return 0
        if k == "no_answer" and r["people"] == 2:
            return 1
        return 2
    rows_sorted = sorted(rows, key=lambda r: (rank(r), r["file"]))
    names = {0: "Rule and VLM disagree", 1: "Rule has no answer, 2 people in crop",
             2: "Everything else"}
    parts = ["<!doctype html><meta charset='utf-8'><title>Facing: rule vs VLM</title>",
             "<style>body{font:14px system-ui;margin:20px;background:#f5f6f4;color:#1b2321}"
             ".g{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));gap:12px}"
             ".c{background:#fff;border:1px solid #ccd;padding:8px}"
             ".c img{max-width:100%;height:150px;object-fit:contain;display:block;margin:0 auto 6px;background:#ddd}"
             "code{font-size:12px}h2{margin:28px 0 10px}.f{color:#a3266b;font-weight:600}</style>",
             f"<h1>Facing classifier: keypoint rule vs VLM</h1><p>{n} crops. "
             f"VLM threshold {args.threshold}. P(facing) uses the node's prompts.</p>"]
    cur = None
    for r in rows_sorted:
        g = rank(r)
        if g != cur:
            if cur is not None:
                parts.append("</div>")
            parts.append(f"<h2>{names[g]} "
                         f"({sum(rank(x) == g for x in rows)})</h2><div class='g'>")
            cur = g
        src = os.path.relpath(os.path.join(args.crops, r["file"]), args.out)
        cls = lambda v: " class='f'" if v == "facing" else ""
        parts.append(
            f"<div class='c'><img src='{html.escape(src)}' loading='lazy'>"
            f"<div>rule: <b{cls(r['kpt'])}>{r['kpt']}</b> · people: {r['people']}</div>"
            f"<div>VLM: <b{cls(r['vlm_node'])}>{r['vlm_node']}</b> "
            f"(P facing {r['vlm_node_pfacing']:.2f})</div>"
            f"<div>hybrid: <b{cls(r['hybrid'])}>{r['hybrid']}</b>"
            + (f" · truth: <b>{html.escape(r['truth'])}</b>" if r["truth"] else "")
            + f"</div><code>{html.escape(r['file'])}</code></div>")
    parts.append("</div>")
    with open(os.path.join(args.out, "report.html"), "w") as fh:
        fh.write("\n".join(parts))
    print(f"\nwrote {args.out}/results.csv and report.html")


if __name__ == "__main__":
    main()
