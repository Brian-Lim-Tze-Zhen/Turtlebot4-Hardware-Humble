#!/usr/bin/env python3
"""table_6_1.py - Table 6.1 of the thesis (facing classification on labelled robot crops).

Usage: python3 table_6_1.py <labels csv> [results.csv]
labels csv: lines "filename,truth" (truth facing / not_facing / no_pair; '#' lines skipped).
results.csv: output of scripts/test_facing_vlm.py (default ../debug_crops_vlm_report/results.csv);
  kpt = keypoint (pose) rule label, vlm_node = MobileCLIP with the node's prompts,
  vlm_talking = MobileCLIP with the "talking" prompt pair.
Prints per truth class: pose rule correct / wrong / no answer, MobileCLIP correct / wrong,
and the talking prompts correct / n. Pose labels side_by_side and diverging count as not facing,
anything else as no answer (same mapping as to_truth_class() in test_facing_vlm.py)."""
import csv, os, sys
here = os.path.dirname(os.path.abspath(__file__))
lab_path = sys.argv[1]
res_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "..", "debug_crops_vlm_report", "results.csv")
truth = {}
for r in csv.reader(open(lab_path)):
    if len(r) >= 2 and not r[0].startswith("#"):
        truth[r[0].strip()] = r[1].strip()
res = {r["file"]: r for r in csv.DictReader(open(res_path))}
missing = [f for f in truth if f not in res]
if missing:
    sys.exit(f"not in results.csv: {missing}")
def pose(k):
    return "facing" if k == "facing" else "not_facing" if k in ("side_by_side", "diverging") else "none"
print(f"labels: {lab_path}  ({len(truth)} crops: " +
      ", ".join(f"{c} {sum(v == c for v in truth.values())}" for c in ("facing", "not_facing", "no_pair")) + ")")
print(f"{'truth':11s} {'n':>3s}  pose c/w/none   MobileCLIP c/w   talking c/n")
for c in ("facing", "not_facing"):
    fs = [f for f, t in truth.items() if t == c]
    p = [pose(res[f]["kpt"]) for f in fs]
    v = [res[f]["vlm_node"] for f in fs]
    tk = [res[f]["vlm_talking"] for f in fs]
    print(f"{c:11s} {len(fs):3d}  {p.count(c):3d} / {sum(x not in (c, 'none') for x in p):2d} / {p.count('none'):2d}"
          f"     {v.count(c):3d} / {sum(x != c for x in v):2d}       {tk.count(c):3d} / {len(fs)}")
