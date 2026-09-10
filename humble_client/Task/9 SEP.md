# Handover — TurtleBot4 socially-aware navigation

Session of 8 Sep 2026 (evening). Paste this as the first message of the next chat.

---

## 0. The one-line state

The Nav2 side is **verified working end to end** — planner, both costmaps, the
predicted-person cloud layer, QoS, the KF, and the SocialCritic were each measured
directly and each behaves correctly. **Everything that still fails traces to
perception:** a static object is published as a person and outlives the real one,
and the camera only sees a person for ~6 s of a 60 s run. Two code changes are
in place and **untested**. Start by testing them.

---

## 1. What was verified WORKING (do not re-debug these)

Each of these was a hypothesis that the data rejected. Measurements are from
`bags/hallway/hallway_7m_06` and `hallway_7m_07`.

| Component | Evidence it works |
|---|---|
| Global planner (NavFn) | `path max cost = 0` while the person sat at cost 100 — plan clears the cloud by 1.0–1.6 m. When the corridor is genuinely blocked it threads inflation at `path max 80–90`, never crossing lethal. That is a 2 m corridor with no alternative route, not a planner defect. |
| Global costmap layer | Probe sampled the costmap at the cloud's own points: `cost 100`, all points in bounds, `z=0.30` well inside the 0–2.0 m filter. |
| Topic plumbing | `/predicted_person_cloud` has 3 subscribers: rviz2, local_costmap, global_costmap. The leading `/` in the config holds; no namespace mismatch. |
| QoS | Publisher and all three subscribers BEST_EFFORT. Matched. |
| Cloud node | 130 "Published cloud" vs 28 safety clears in the last run; 611 cloud msgs in the bag. |
| Fusion binding logic | 18 successful `pair-bearing` pairings vs 2 `no lidar anchor` failures. |
| Phantom defences | `Dropped stable 1 — never got lidar corroboration within 4.0s ... Blacklisting bearing -5.5 deg for 15s` fired correctly. |
| KF / cloud rate | 10 Hz, populated whenever fusion has a track. |

**Rejected hypotheses, each disproved by measurement** (don't revisit without new
evidence): NavFn needs replacing · costmap range 6.0 m was the limit · topic name
mismatch · QoS mismatch · cloud node dead · `CAMERA_CORROBORATION_TIMEOUT` too
short · clipped-bbox range poisoning cluster choice · orphan re-ID unreachable.

---

## 2. What was actually FIXED today

### `min_publish_conf` 0.75 → 0.45 in `scripts/yolo_leg_detector_lidar.py`
The single change that moved a number. YOLO runs at `conf=0.15` but nothing was
published below 0.75, and a person cropped to knees-and-boots never scores that.

| | 06 (conf 0.75) | 07 (conf 0.45) |
|---|---|---|
| duration | 50.7 s | 61.2 s |
| camera detections | 66 | 78 |
| **fused** | **10 (0.20 Hz)** | **118 (1.93 Hz)** |
| fused max gap | died entirely | **1.05 s** |
| SocialCritic | mostly inert | 37 active / 27 inert |

**~10× fused output.** Note the camera *rate* barely changed — the gain is that
detections now spread across the run instead of bunching, keeping bindings alive.

### Nav2 config (`config/social_nav2_ablation_E_critweight20_socialcritic_on.yaml`)
- `predicted_person_cloud` in the **global** costmap: `obstacle_max_range` and
  `raytrace_max_range` 6.0 → **9.0**. The person is fused out to 8.8 m; 6.0 m
  discarded them before marking. (The **local** block stays at 4.0 — the local
  costmap is 7×7 m rolling, so 3.5 m radius; more range there does nothing.)
- `bt_navigator`: removed the duplicate `odom_topic: odom` that overrode
  `odom_reliable` on the line above. Backup `.bak_odomdup2`.

### `odom_qos_relay.py` — it was never running
`/turtlebot4/odom_reliable` did not exist, so `bt_navigator` **and**
`controller_server` were subscribed to a dead topic — MPPI had no odometry.
`python3 /root/thesis_social_navigation_ws/odom_qos_relay.py` must run before Nav2.
**Add it to the bringup launch** — this silently invalidates trials, same failure
class as the SocialCritic topic name.

### Tooling
`config/cloud_qos.yaml` — QoS override so `ros2 bag record` can capture the
BEST_EFFORT `/predicted_person_cloud`. Without it the topic records zero messages.

---

## 3. The two real problems

### A. A static object is published as a person and outlives the real one

`hallway_7m_07`, fused output split by identity:

```
sid /1  lidar_only        n=72  t=23.4-32.3  span=0.94m  med=(-0.94,  9.64)   <-- 61% of all output
sid ;2  camera_confirmed  n=17  t=24.2-26.4  span=3.01m  med=(-2.17,  8.57)   <-- you
sid <2  camera_confirmed  n=16  t=23.0-26.7  span=3.01m  med=(-4.21, 10.77)   <-- you, split identity
```

A 0.94 m span over 9 s is not a person. That coordinate recurs across sessions:
`(-1.21, 9.45)`, `(-1.29, 9.52)`, `(-0.890, 9.793)`, `(-1.16, 0.12)` in the calib
bags. The costmap is being told a person stands where a wall is.

**Why it qualifies as a person — and why raising `static_move_threshold` cannot fix it.**
Per published lidar track in that bag:

| id | span | max per-scan step | median pos |
|---|---|---|---|
| 16 (**the person**) | **7.26 m** | 0.691 m | (-4.49, 12.25) |
| 1 (static) | 1.02 m | 0.636 m | (-1.30, 9.75) |
| 0 (static) | 2.06 m | 0.676 m | (-1.64, 10.14) |
| 5 (static) | 0.21 m | 0.174 m | (-0.55, 7.10) |

Per-scan step **does not separate them** — 0.636 vs 0.691 m. Any threshold that
rejects the wall rejects the person. Those 0.6 m jumps are not motion: the
*clustering* changes what it considers one object between scans (wall corner, door
frame, passing legs), so the centroid hops. Same track id, different physical thing.

**Cumulative span from birth does separate them: 7.26 m vs 0.21–2.06 m.** Jitter
oscillates around a fixed point and stays bounded; walking accumulates.

### B. The camera sees a person ~6 s in a 60 s run

`/person_positions_map` active window: t=12.3–17.8 in run 06 (5.5 s), t=20.9–26.7
in run 07 (5.8 s). **Lowering the confidence gate densified this window tenfold but
did not lengthen it by a second.** Two runs, two configs, same ~6 s.

Frames extracted at t=17.45/17.72/17.99/18.25 show why: the lens is pitched at the
floor. All four frames are dominated by reflective tile; the person is visible from
roughly the knee down and exits the right edge. Every bbox has `y1=0..2`.

Root fix is mounting geometry — **tilt the camera up**. Brian has stated this is
currently not possible. Until it is, this is a stated limitation, and any gate tuned
against clipped bboxes is tuned against a biased input (circular, as the previous
handover warned).

---

## 4. Two changes IN PLACE and NOT YET TESTED

Both compile. Neither has a valid before/after measurement.

### `confirm_min_span = 1.5` in `scripts/lidar_person_detector.py`
Adds `birth_x`/`birth_y`/`span` per track; `confirmed` now requires **both** the
original recent-motion test **and** ≥1.5 m cumulative travel from birth.
`confirm_min_span:=0` disables. Backup `.bak_span_added`.
Chosen because 1.5 m sits in the empty gap between 7.26 m (person) and 2.06 m
(worst static).
**Known cost:** at 1.3 m/s, 1.5 m is ~1.2 s of walking before a person is published
— a real bite out of an already-short window, and a hesitant walker may never
accumulate it. Tune down toward 1.0 m if detection is too late.

### `USE_KEYPOINT_BEARING = True` in `scripts/identity_fusion_node_lidar.py`
`yolo_leg_detector_lidar.py` already publishes COCO keypoints 13–16 (knees, ankles)
in `parts[6]`; they were documented in the format comment but **never parsed**.
Bearing now comes from the two lowest keypoints (ankles) instead of the bbox
horizontal centre, falling back to the bbox when no keypoint is present.
Rationale: ankles sit at the RPLIDAR's own scan height, so their bearing refers to
the *same physical feature* the lidar clusters on — and they stay visible in a
floor-pointed frame when the torso does not. The bbox centre of a knees-down box
drifts with stance. Backup `.bak_kptbearing`.
Both `pair-bearing` log lines now print `kpt2`/`kpt1`/`bbox` so you can see which
source was used.

### Also present, no measured effect (harmless, keep)
`BBOX_CLIP_TOP_PX = 2.0` — a bbox with `y1<=2` reports `bbox_range=None` rather
than a wrong range. Logically right; changed nothing in replay.
`REID_ON_LIDAR_MOTION = False` — a documented dead end. First version used a 0.08 m
per-scan guard and produced a phantom republishing one point for 2.3 s; second used
1.5 m cumulative span and never fired because **`orphaned_identities` was empty for
the whole run**. Why identities are never orphaned on lidar-id fragmentation is an
open question — the existing `find_orphan_to_reattach()` is only reachable from
`camera_callback`.

---

## 5. TOMORROW — do these in order

### Step 0 — before anything else
```
python3 /root/thesis_social_navigation_ws/odom_qos_relay.py
```
Leave running in its own terminal, **then** start Nav2. Verify:
```
ros2 topic info -v /turtlebot4/odom_reliable | grep -E "count|Reliability"
```
Want: 1 RELIABLE publisher, 2 subscribers after Nav2 is up. If subscribers are 0,
Nav2 started first — restart it.

### Step 1 — test keypoint bearing ALONE
Disable the span check so only one variable changes. Launch the detector with:
```
-p confirm_min_span:=0.0
```
Then walk the hallway with a goal active and record:
```
cd ~/thesis_social_navigation_ws && ros2 bag record --qos-profile-overrides-path config/cloud_qos.yaml -o bags/hallway/hallway_7m_08 /turtlebot4/scan /turtlebot4/odom /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/map /turtlebot4/amcl_pose /turtlebot4/cmd_vel /turtlebot4/cmd_vel_nav /turtlebot4/plan /turtlebot4/global_costmap/costmap /turtlebot4/local_costmap/costmap /turtlebot4/oakd/rgb/preview/image_raw/compressed /turtlebot4/oakd/rgb/preview/camera_info /turtlebot4/person_markers /lidar_person_clusters /person_positions_map /person_positions_fused /predicted_person_positions /predicted_person_cloud
```
First check — are keypoints even being used?
```
grep -o "kpt[0-9]\|bbox)" ~/thesis_social_navigation_ws/perception_bringup_output.log | sort | uniq -c
```
If it is mostly `bbox`, keypoints aren't reaching fusion — check
`leg_kpt_min_conf = 0.3` in the YOLO node, which may be filtering them out on a
cropped body. That would be the thing to fix rather than the bearing logic.

### Step 2 — then test `confirm_min_span` ALONE
Same route, `confirm_min_span:=1.5`, record as `hallway_7m_09`.

### Step 3 — judge both against these baselines (`hallway_7m_07`)
```
duration 61.2 s · clusters 568 · camera 78 · fused 118
phantom (-0.94, 9.64) held 72 of 118 fused messages (61%)
```
Success = the phantom's share drops sharply **and** the person's tracks get longer.
Failure mode to watch = person's cluster count collapses (span gate too strict).

The analysis one-liners for both checks are in this chat; ask for them by name —
"fused split by identity" and "cluster span table".

### Step 4 — only after 1–3, re-test 7 m avoidance
`probe_both.py` while walking from 7 m. This is the first time the input will be
good enough for that test to mean anything.

---

## 6. Standing instructions (carried forward + new)

- **Replay does NOT reproduce live behaviour for these nodes.** Replay uses recorded
  TF via `--remap /turtlebot4/tf:=/tf`; live uses the real tree with AMCL. Different
  transforms → different map-frame positions → different clustering and ids. Three
  conclusions tonight were drawn from replay-vs-live comparisons and all three were
  invalid. **Compare live against live.**
- Establish ground truth before analysing. Every metric today was a proxy.
- Check what a metric measures before reporting it. `ros2 bag info` "Duration" is
  first-to-last message *in that bag* — a bag holding only `/person_positions_fused`
  showing 3.8 s means fusion published for 3.8 s, not that recording was short.
- One change at a time, measured on the same route before and after.
- Note the measurement convention in bag notes. Distance from *what* to *what*.
- Replay at 1×. Rate-dependent checks make any other speed meaningless.
- Back up **before** editing, not after. One revert failed tonight because the
  backup command never ran.

---

## 7. Data

Bags now organised under `bags/`: `calib/ hallway/ trials/ replays/ perception/`.

- `hallway_7m_07` — **the reference run.** 61.2 s, conf 0.45, all topics incl. cloud.
- `hallway_7m_06` — 50.7 s, conf 0.75. The before-run for the confidence change.
- `hallway_7m_02` — 97 s, 2528 clusters, 538 fused. The best raw numbers of the set;
  never explained why. Worth revisiting.
- `hallway_7m_05` — worst run (110 clusters), but the first with `/predicted_person_cloud`.
- `calib_2m` / `calib_3m5` / `calib_5m` — **cluster output is furniture-dominated.**
  Recorded before the ray-consistency fix; the same static coords appear in all
  three. Usable for raw scan analysis, not for detection-rate curves as-is.
- `hallway_7m_01/03/04` — earlier hallway runs, low fused rates.

Reference poses for the 2 m hallway:
```
START x=-1.286 y=8.291 qz= 0.932 qw=0.362
GOAL  x=-6.839 y=13.278 qz=-0.354 qw=0.935     (7.46 m apart, goal 0.5° off the nose)
```
AMCL covariance ~0.20–0.22 on x/y in this corridor (±0.46 m). The cloud is placed in
the map frame, so localisation error moves the marks. Suspect this first if reaction
distance varies between otherwise identical runs.

---

## 8. Known issues, not fixed

- **76 `Transform data too old when converting from map to odom`** (controller_server)
  appeared in the last Nav2 log and not before. Timestamps show data ~4 s ahead of
  the available transform. Unexplained; could undermine everything above. Check
  first vs last occurrence to see whether it is startup-only.
- **Detector TF failures changed character** — now `extrapolation into the past`
  (26×) rather than the tree-disconnected startup errors. Usually AMCL lag or a
  short buffer; drops clusters when it fires.
- `show_debug_image = True` in the YOLO node runs `cv2.imshow` every frame. Pure
  overhead on the Pi during recorded trials; candidate contributor to the 22
  control-loop misses at 20 Hz.
- `process_every_n_frames = 2` halves the effective camera rate before anything else.
- **ByteTrack fragmentation** — one person split across `sid ;2` / `sid <2`
  simultaneously. Upstream of fusion.
- **Constant global rerouting** — cloud is in the global costmap, `ComputePathToPose`
  RateController raised 1 → 5 Hz. Design tension unresolved.
- **Map completeness ceiling** — 44.7% of returns land on cells `lab2_map` marks
  occupied. No map veto can reject an obstacle the map never recorded. State as a
  limitation.
- **Periodic safety clear every 3 s** on both costmaps. Observed `cost>0` dropping
  99 → 48 with the cloud unchanged. Real, measured, not yet addressed.
