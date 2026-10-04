<!-- Append below Result 11 in RESULTS.md. Bag rows go into the bag inventory. -->

## Result 12 — Head-on with robot navigating: INHERIT_S, coast, ellipse length

**Claim.** Three changes, each tested live against proto_L2 on the same
corridor section (config E, cloud radius 0.4 m, robot on a Nav2 goal,
person starting far and walking head-on):

1. `INHERIT_S` 1.0 → 0.3 s removes stale range inheritance: pass rate 44 → 80 %, 3 → 1 stable id.
2. Coasting along predicted velocity removes the frozen cloud: the cloud follows the person, and the left-wall squeeze (~0.1 m in L2) is gone.
3. `ELLIPSE_A_SLOPE` 0.50 → 0.75 does not change clearance, but removes reversing and spinning during the detour.

### How each metric is defined (read before the tables)

| Metric | Source | Meaning |
|---|---|---|
| Pass rate | bag counts `/camera_ray_clusters` ÷ `/person_positions_map` | share of YOLO detections that got a LiDAR range |
| Stable ids | `headon_metrics.py` | identities published for one person; 1 is ideal |
| Last pipeline distance | `distance profile`, t ≥ 0 | robot centre → estimated person centre when the camera lost the person |
| **Person pass** | `nearest UNMAPPED LiDAR point`, side=right | robot centre → nearest **leg surface** (ground truth, pipeline-independent); taken where the bearing sweeps front → side → back |
| Clearance | pass − 0.189 m | robot edge → leg surface |
| Front minimum | `nearest LiDAR return ±30°` | includes walls; values pinned at ±30° are the corridor wall at the sector edge |
| Stops | cmd_vel ≈ 0 for ≥ 0.5 s | also re-run at ≥ 0.2 s |
| Lateral deviation | driven path vs start→end line | how far the robot bent away |
| vx / wz | `/turtlebot4/cmd_vel`, t+0 → t+15 | forward speed (m/s, negative = reversing) and turn rate (rad/s) |

**Excluded from every table.** `closest approach` in the pipeline
section: in H4–H6 it falls at t < 0 (person standing near the robot
before motion started), i.e. setup, not the test. A steady ~0.8–0.95 m
unmapped return at about −60 to −100 deg from t+0 is a fixed object
beside the path (present in all three runs, does not sweep).

### A. INHERIT_S 1.0 → 0.3 s (proto_H4)

| | L2 | H4 |
|---|---|---|
| Pass rate | 46/105 = 44 % | **99/124 = 80 %** |
| Stable ids | 3 | **1** (camera switches reattached) |
| Refusals | 7, one stuck ~3.6 s | **3**, none stuck |

L2's stale inherit (3.11 m from an id seen 0.87 s earlier, person at
6.5 m) cannot occur with a 0.3 s window. **Committed.**

### B. Coast along predicted velocity (proto_H4, H5)

Log evidence, cloud node output after camera loss:

| run | cloud while coasting | then |
|---|---|---|
| L2 | frozen at (−9.01, 14.51) for ~1.9 s | — |
| H4 | (−10.82, 16.00) → (−12.62, 17.29), ~0.26 m per 0.2 s | held ~1 s, pruned |
| H5 | (−10.56, 16.38) → (−13.04, 17.93); later id 1 (−8.16, 13.88) → (−7.63, 13.46) | held ~1 s, pruned |

Hold ~1 s = KF `coast_timeout` 1.5 s ends, cloud `TRACK_TIMEOUT` 1.0 s
expires.

| | L2 | H4 | H5 |
|---|---|---|---|
| Left wall | **~0.1 m** | not squeezed | front min 0.62 m |
| Stops (≥ 0.5 s) | 0 | 0 | 0 |
| Lateral deviation | 0.22 m | 0.47 m | 0.35 m |
| Stable ids | 3 | 1 | 2 |
| Person pass (right) | ~1.0 m | 0.61 m @ t+11.3 | 0.74 m @ t+9.7 |

H5's second id: id 0 was still coasting 1.3 m from where the person was
re-detected, beyond reattach distance. The same person, inside the
1–2 id pass rule. **Committed.**

### C. ELLIPSE_A_SLOPE 0.50 → 0.75 (proto_H6 vs H5)

Lane half-length `a = 0.60 + slope × speed`, placed entirely ahead of
the person. At 1.0 m/s the lane ahead goes 2.2 → 2.7 m; width unchanged
(b = 0.4). Cloud size while walking: max 82 points (H4) → 114 (H6).

| | H5 (0.50) | H6 (0.75) |
|---|---|---|
| Stable ids | 2 | **1** |
| Camera lost at | 2.41 m | 3.70 m |
| Person pass (right) | 0.74 m (clearance 0.55) | 0.69 m (clearance 0.51) |
| Front minimum | 0.62 m | 1.00 m (end of run) |
| Stops ≥ 0.5 s / ≥ 0.2 s | 0 / 0 | 0 / 0 |
| Lateral deviation | 0.35 m | 0.30 m |
| **Min vx** | **−0.31 m/s** (t+6.0) | 0.18 m/s during the pass |
| **Max \|wz\|** | **1.50 rad/s** (t+6.5, t+8.6) | 0.78 rad/s |
| Mean vx, t+0 → t+15 | 0.19 m/s | **0.25 m/s** |

H5 during the pass (t+6 → t+10): reversed at −0.31 m/s while turning at
−1.50 rad/s, stood still turning at t+7.6, then crawled at 0.04–0.15 m/s
turning +1.50 rad/s. The stop counter reports 0 because reversing and
turning on the spot are not "still". H6 never reversed; vx stayed
0.18–0.31 m/s for the whole run.

**Interpretation.** The pass distances 0.61 / 0.74 / 0.69 m are within
run-to-run spread; they depend on the line the person walks. The
measurable effect of the longer lane is on motion quality: the bend
starts earlier and is continuous instead of a late reverse-and-spin.
**Committed on motion quality, not clearance.**

### Limits

- **One run per condition.** H5 vs H6 motion quality is a single pair; a
  repeat at 0.75 would confirm it is not the walking line.
- Person pass is still below L2's ~1.0 m in all three runs. L2's pass was
  obtained by pushing the robot to ~0.1 m from the wall, so it is not a
  target to match.
- The `left` unmapped check excludes mapped walls, so it is **not** a
  wall-clearance measure (H4 left 0.46 m @ +132 deg is an unmapped
  object). Wall proximity is only bounded here by the ±30° front sector;
  a mapped-wall distance metric is missing.
- vx/wz were compared for H5 and H6 only, not H4 or L2.
- Pass rate from bag counts was computed for H4 only.
- Leg surface ≠ body centre: centre-to-centre is roughly +0.10–0.15 m (not measured).

### Bag inventory (add rows)

| bag | content |
|---|---|
| proto_H4_headon_coast | head-on, INHERIT_S 0.3 + coast; 80 %, 1 id, pass 0.61 m |
| proto_H5_headon_coast | repeat of H4, no changes; 2 ids, pass 0.74 m, reverse+spin during detour |
| proto_H6_headon_ellipse075 | + ELLIPSE_A_SLOPE 0.75; 1 id, pass 0.69 m, no reversing, mean vx 0.25 |
