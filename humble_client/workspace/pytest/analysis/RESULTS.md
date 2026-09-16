# Perception front-end: results log

Running record of measured results for the camera-ray person detection
work. Append new results at the end; do not rewrite earlier entries —
if a result is later shown wrong, add a correction below it rather
than deleting it.

Scripts live in `pytest/analysis/`. Bags in `bags/`.

---

## Result 1 — Ray selection rule and window width

**Claim.** Taking the *minimum* range within a ±6° window around the
camera bearing is the most stable way to convert a camera bearing into
a LiDAR range. It roughly halves range instability versus the original
±3° window, and beats median, nearest-to-centre, and cluster-based
selection on every configuration tested.

**Method.** For each camera detection in a bag, the leg-keypoint
bearing is computed (pinhole, fx 201.308, cx 124.95, yaw offset
2.25°), the `/turtlebot4/scan` returns within ±half-width of that
bearing are collected, and one is selected by each candidate rule.
Two metrics:

- **>0.3 m %** — fraction of consecutive detections whose range
  changed by more than 0.3 m. A person cannot move that far between
  frames at these rates, so this counts selection failures (the ray
  slipping onto a background return).
- **range sd** — standard deviation of selected range over the run.
  Partly contaminated by the person's real movement, so it is a
  secondary metric, not decisive on its own.

**Data.** Two bags recorded 14 Sep 2026, robot parked, one person:

| bag | duration | camera detections |
|---|---|---|
| `debug_inputs_20260914_161006` | 101.7 s | 1108 |
| `debug_inputs_20260914_160801` | 69.2 s | 738 |

161006 was used to choose the parameter; **160801 was held out** and
used only to check it. The result held on the held-out bag with a
larger margin than on the tuning bag, which is the main reason to
trust it.

**Results, tuning bag (161006):**

| rule | ±3° | ±6° | ±10° |
|---|---|---|---|
| min | 2.1% / 0.402 m | 1.3% / 0.338 m | 3.7% / 0.169 m |
| median | 2.9% / 0.513 m | 2.7% / 0.503 m | 2.3% / 0.352 m |
| nearest-centre | 2.8% / 0.509 m | 2.8% / 0.509 m | 2.8% / 0.509 m |
| cluster | 3.3% / 0.564 m | 4.9% / 0.874 m | 6.3% / 0.998 m |

**Results, held-out bag (160801):**

| rule | ±3° | ±6° | ±10° |
|---|---|---|---|
| **min** | 6.0% / 0.168 m | **0.0% / 0.040 m** | 0.0% / 0.030 m |
| median | 6.2% / 0.200 m | 6.2% / 0.196 m | 0.3% / 0.099 m |
| nearest-centre | 4.6% / 0.228 m | 4.6% / 0.228 m | 4.6% / 0.228 m |
| cluster | 5.4% / 0.227 m | 5.2% / 0.226 m | 6.2% / 0.276 m |

**Decision.** `RAY_HALF_WIDTH_DEG` 3.0 → **6.0**, selection rule stays
`min`. ±10° scores marginally better again but the gain over ±6° is
small, and a wider window admits more of the scene — with a second
person or clutter inside the window, min-range will take whatever is
nearest regardless of whether it is the tracked person. 6° keeps the
margin without that exposure.

**Caveats — read before citing.**

- Tuned on single-person data at 1–1.6 m range, robot stationary. A
  fixed angular window subtends different physical widths at different
  ranges (a 0.5 m-wide person is ~28° at 1 m but ~7° at 4 m), so this
  value is not expected to transfer to long-range or multi-person
  scenes. A range-adaptive window is the principled fix and is not
  implemented.
- `min` beating `median` and `cluster` was not the expected outcome
  and contradicts an earlier working hypothesis in this project that
  min-range selection was the cause of the observed range bistability.
  It is consistent across both bags, so the earlier hypothesis is
  retired — but it is worth re-testing on any bag with real clutter.
- Median jitter is 0.000 m for every rule, i.e. the typical
  frame-to-frame change is below LiDAR resolution. The rules differ
  only in how often they break, not in normal operation.

**Reproduce:**

```
cd ~/thesis_social_navigation_ws
python3 pytest/analysis/rules.py bags/debug_inputs_20260914_161006/rosbag/*.db3
python3 pytest/analysis/rules.py bags/debug_inputs_20260914_160801/rosbag/*.db3
```

---

## Result 2 — Stationary person detection (architecture comparison)

**Claim.** The LiDAR-primary pipeline cannot detect a person who is
standing still, and the camera-primary pipeline can. This is a
structural property of the two designs, not a tuning difference.

**Why it is structural.** A 2D LiDAR has no feature that separates a
person from furniture: measured on this robot, a wall fragment of
0.061 m width sits *between* two real leg clusters at 0.048 m and
0.100 m, and a 2-point cluster is collinear by definition so no shape
test can exist. Motion is therefore the only available discriminator,
which is why `lidar_person_detector.py` gates publication on
`static_move_threshold` (0.1 m over 0.3 s). A person who is not
moving fails that gate by construction. The camera-ray node never
asks the LiDAR to classify — the camera classifies, and the LiDAR only
supplies range along the resulting bearing — so the motion gate does
not exist in that design.

**Part A - quantified: camera-primary coverage of a stationary person.**

Measured with `coverage.py`. Stationary segments are derived from the
camera-ray position stream itself (maximal runs >=3.0 s where the
published map position stays within a 0.25 m box), then coverage is
the fraction of 0.5 s bins in each segment containing at least one
published position.

| bag | stationary time | segments | camera-ray coverage |
|---|---|---|---|
| `debug_inputs_20260914_160801` | 36.7 s | 1 | **100.0%** |
| `debug_inputs_20260914_161006` | 77.4 s | 8 | **95.0%** |
| combined | 114.1 s | 9 | ~96.6% |

In 161006, five of the eight segments were at 100%; the two weakest
were 33.3% (3.4 s) and 81.8% (5.8 s), both late in the bag. Both were inspected
(see Result 3) and are NOT range bistability: each coincides with a
YOLO detection gap of almost identical length (2.30 s vs 2.32 s;
1.27 s vs 1.30 s). The camera-ray node had no input during those
windows.

Supporting single-instance detail, person standing at ~1.3 m, robot
parked: continuous publication for 14 s, one camera track id held
throughout, map position stable at (-7.22, 15.62) +/- 0.01 m, range
1.29-1.30 m.

**Part B - structural, not measured on these bags: why the LiDAR-primary
pipeline cannot do this.**

IMPORTANT LIMITATION: `/lidar_person_clusters` is EMPTY in both debug
bags (0 messages). The camera-ray node was run standalone against YOLO
and `/scan`; the full perception bringup was not running during those
recordings. The `lidar(conf1)` and `lidar(all)` columns printed by
`coverage.py` therefore read 0.0% because there is no data, NOT
because a running detector was measured at zero. Do not cite those
columns as a measurement.

The comparison is still supportable, from two other sources:

1. *Code path.* `lidar_person_detector.py` gates publication on
   `static_move_threshold` (0.1 m over 0.3 s). A person who is not
   moving fails that gate by construction, so non-detection is a
   property of the design rather than a tuning outcome.
2. *Logs from the same session.* The detector's own output on
   stationary people reads
   `(idle) N static/non-moving candidate(s) tracked, nothing publishing`
   - i.e. it is tracking the candidate and withholding it. This is
   direct evidence of the mechanism, timestamped, from the same robot
   and hall on the same day.

**What would make this a like-for-like result.** One re-record with
BOTH pipelines running simultaneously, same walk sequence, so
`coverage.py` produces both columns from real data. This is the single
highest-value use of any further hardware time.

---

## Result 3 - Attribution of output gaps: camera stage vs ray logic

**Claim.** Most lost tracking time is inherited from the camera
(YOLO) stage, not introduced by the camera-ray conversion. Across both
bags, 85% of gap time coincides with a simultaneous gap in
`/person_positions_map`.

**Method.** `gaps.py` finds every gap >0.5 s in `/camera_ray_clusters`
and checks whether a `/person_positions_map` gap overlaps the same
window. Overlap = INHERITED (the node had no input); no overlap = OWN
(ray selection, range guard, or TF dropped it).

| bag | cam-ray gaps | YOLO gaps | total gap | inherited | own |
|---|---|---|---|---|---|
| `160801` | 3 | 8 | 1.8 s | 1.8 s (100%) | 0.0 s (0%) |
| `161006` | 10 | 9 | 13.8 s | 11.4 s (83%) | 2.4 s (17%) |
| combined | 13 | 17 | 15.6 s | 13.2 s (85%) | 2.4 s (15%) |

**Secondary observation.** In 160801, YOLO had 8 gaps but camera-ray
only 3 - five upstream gaps were short enough that the next detection
still fell inside the same coverage bin. The node does not amplify
upstream loss.

**Consequence for Result 2.** The ~5% coverage shortfall on stationary
people is mostly a camera-stage problem, not a property of the
camera-ray method. The more precise claim is: *camera-ray converts
camera detections into stable positions with little additional loss;
residual coverage loss is inherited from the detector upstream.*

**Consequence for the camera stage - REVISED, see Result 4.** The
"inherited" loss was initially assumed to be an unexplained camera-stage
fault. Result 4 shows it is not: it is the documented close-range /
field-of-view limitation, appearing a second time. The camera hardware
and the YOLO node were both working normally throughout.

**Caveat.** 0.5 s gap threshold and 0.5 s coverage bins are arbitrary
and interact - a gap slightly under threshold still costs coverage.
Quote the threshold with any number from this script.

**Reproduce:**

```
cd ~/thesis_social_navigation_ws
python3 pytest/analysis/gaps.py bags/debug_inputs_20260914_160801/rosbag/*.db3
python3 pytest/analysis/gaps.py bags/debug_inputs_20260914_161006/rosbag/*.db3
```

---

## Result 4 - The "YOLO dropout" is the publish threshold, not a fault

**Claim.** The camera-stage gaps in Result 3 are not a camera failure
or a YOLO stall. The camera streamed continuously, YOLO detected and
tracked the person throughout, and publication stopped because
confidence fell below `min_publish_conf = 0.45` as the person filled
and exited the frame at close range.

**Evidence.** The 5.33 s gap at t=77.2 s in `debug_inputs_20260914_161006`:

- `/turtlebot4/oakd/rgb/preview/image_raw/compressed` delivered 151
  frames during the gap at 28.3 Hz - full rate, uninterrupted. Same for
  the 2.32 s gap (70 frames, 30.1 Hz) and the 1.30 s gap (39 frames,
  30.1 Hz). The camera was alive.
- The last published detections before the gap decay smoothly into the
  threshold while the bbox grows and reaches the frame edge:

| t | conf | bbox | width |
|---|---|---|---|
| 284.39 | 0.59 | (119,0,226,196) | 107 px |
| 284.46 | 0.57 | (121,0,232,196) | 111 px |
| 284.52 | 0.45 | (124,0,240,200) | 116 px |
| 284.61 | 0.45 | (124,0,245,208) | 121 px |

  x2 reaches 245 on a 250 px frame. The person walked up to the camera
  and out of the right edge; confidence crossed 0.45 exactly there.

**Two-stage thresholding (important - this was initially misread).**
`yolo_leg_detector_lidar.py` runs the model at `conf=0.15` (line 115)
and applies `min_publish_conf = 0.45` (line 64) separately in Python.
So detections between 0.15 and 0.45 ARE produced and ARE tracked by
ByteTrack - they are discarded before publication. The low model
threshold is deliberate: it keeps the tracker fed so a person who dips
in confidence keeps their track id instead of being re-assigned. That
is a sound design and should not be raised.

**Confidence does not order person-ness at close range.** From the
published distribution over 1181 detections in 161006:

- 115 detections (9.7%) sit within 0.10 of the publish threshold.
- The six widest boxes - 180-183 px on a 250 px frame, i.e. a person
  nearly filling the view, all with 4 leg keypoints - score only
  0.51-0.56.
- Separately documented in `identity_fusion_node_lidar.py`: YOLO-pose
  scored a door frame's vertical jambs as a person at **0.76-0.86**,
  with all 4 leg keypoints, repeatedly.

So a real person at 1 m can score 0.45 while a door frame scores 0.86.
Confidence is tracking how typical the view is, not whether a person is
present. Lowering `min_publish_conf` would recover real close-range
detections AND admit more of the door-frame class; the two are not
separable by confidence alone.

**Implication.** The threshold is the wrong lever. The underlying
problem is that the camera cannot usefully see a person at ~1 m,
which is the same close-range/FOV limitation identified independently
earlier. The root fix named in the code is mounting geometry (tilt the
camera up); the architectural alternative is to stop requiring camera
confirmation at close range and rely on LiDAR continuation there.

**Open question - now answered, see Result 5.** How many detections sit
in 0.15-0.45: 24 against 1066 published, 83% of them carrying all four
leg keypoints.

**CONFIRMED for the condition-C run.** The 14 Sep condition-C attempt
(`logs/perception_C_20260914_131511.log`) shows the same signature, so
its ~100 s "YOLO outage" was also the FOV limit, not a fault. Track 24
crosses the frame left-to-right - x1 36 -> 52 -> 74 -> 95 -> 108 -> 113,
bbox width 84 -> 106 px, y1=0 throughout - while confidence slides
0.74 -> 0.71 -> 0.68 -> 0.52 -> 0.47, stopping one step above the 0.45
gate. Detections never resumed because the person did not re-enter the
frame usefully during that run.

This means the condition-C failure chain (camera silent -> fusion
starves -> cloud coasts on a stale track -> SocialCritic inert -> robot
stops) is correct in every link except the first: the camera stage did
not fail, it reached the edge of its operating envelope. The whole
day's findings reduce to one limitation rather than two.

**Reproduce (camera-alive check):**

```
# per-gap frame counts during YOLO silences
python3 pytest/analysis/gaps.py bags/debug_inputs_20260914_161006/rosbag/*.db3
# then inspect the matching perception log around the gap timestamps
grep "yolo_bytetrack" logs/debug_inputs_20260914_160954.log | less
```

---

## Result 5 - How much is lost below the publish threshold

**Claim.** Lowering `min_publish_conf` would recover very little.
Across a full replay of `debug_inputs_20260914_161006`, only 24
detections fell in the 0.15-0.45 band against 1066 published - a
ceiling of about 2.2% more detections. It does not close the
close-range gaps.

**Method.** `yolo_leg_detector_lidar.py` was instrumented to log
rejected detections instead of silently dropping them (two log lines,
one on each reject path - `conf < min_publish_conf` and
`track_id < 0`). The bag was then replayed with `--clock` and the
node run against it with `use_sim_time:=true`. No model change, no
hardware.

**Results.**

| outcome | count |
|---|---|
| published (conf >= 0.45, track assigned) | 1066 |
| rejected: conf below threshold | 24 |
| rejected: `track_id < 0` (unassigned) | **0** |

Confidence distribution of the 24:

| band | n |
|---|---|
| 0.20 | 2 |
| 0.30 | 13 |
| 0.40 | 9 |

range 0.20-0.44. Leg keypoints among them: **20 of 24 had all four**;
the rest had 0, 2 and 3. Median bbox width 90 px, max 184 px on a
250 px frame - i.e. these are close-range detections.

**Interpretation.**

1. *The threshold is not where the loss is.* 24 detections spread over
   ~100 s is roughly one per 4 s. The 5.33 s gap of Result 4 is not
   recoverable this way, because during that window YOLO produced
   nothing at all - not even below 0.45. This retires the idea that
   lowering the publish threshold would fix close-range coverage.
2. *But the rejected ones do look real.* 83% (20/24) carried all four
   leg keypoints. The model is locating legs confidently while scoring
   the detection as a whole low - consistent with Result 4's finding
   that confidence tracks how typical the view is rather than whether
   a person is present. Small in number, but the direction is
   unambiguous.
3. *The `track_id < 0` gate never fired on this bag.* Worth knowing
   before anyone spends time on it: on this data it discards nothing.

**Caveat - replay is not frame-exact.** The replay produced 1066
published detections against 1181 in the original live run, about 10%
fewer, consistent with frames being dropped under different CPU load.
ByteTrack also carries state, so track ids differ between runs. These
counts are reliable in magnitude, not per-frame. Any figure quoted
from this experiment should carry that qualification.

**Decision.** Leave `min_publish_conf` at 0.45. The measured upside is
~2% more detections, against admitting more of the door-frame class
documented in Result 4 (scored 0.76-0.86, i.e. already above any
threshold you would set). The close-range problem is geometric, not a
threshold choice.

**Reproduce:**

```
# terminal 1
ros2 bag play ~/thesis_social_navigation_ws/bags/debug_inputs_20260914_161006/rosbag --clock
# terminal 2  (node must carry the REJECT logging patch)
python3 ~/thesis_social_navigation_ws/scripts/yolo_leg_detector_lidar.py \
  --ros-args -p use_sim_time:=true \
  -r /oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed \
  2>&1 | tee logs/rejectlog_161006.log
```

Backup of the un-instrumented node:
`scripts/yolo_leg_detector_lidar.py.bak_rejlog`.

---

## Result 6 - The occlusion guard no longer pays for itself

**Claim.** The range-jump guard in `camera_ray_person_node.py` was
compensating for an under-sized search window. With the window
corrected to +/-6 deg (Result 1) it provides no measurable benefit on
one bag and 0.17 percentage points on the other, while introducing a
failure mode in which it latches onto a stale reference and rejects
valid readings for several consecutive frames. Recommendation: remove
it.

**Background.** The guard compares each new ray range against the last
ACCEPTED range for that camera id and rejects anything further than
`MAX_RANGE_JUMP_M + MAX_RANGE_SPEED * dt`. Observed live on 14 Sep:
cam:11 latched at 1.68 m and then rejected 3.94, 4.42, 3.65, 3.62 for
roughly a second, then rejected the way back down (3.94 -> 1.04) - the
reference itself was wrong, so recovery was blocked.

**Method.** `guard.py` replays the bag, reproduces the raw ray
measurement for each camera detection at +/-6 deg, then applies five
guard designs to the same measurement stream:

- `none` - no guard
- `last-acc` - current design, reference = last accepted range
- `last-raw` - reference = last raw measurement, accepted or not
- `median3` - reference = median of last 3 raw measurements
- `hysteres` - last-raw, force-accept after 3 consecutive rejects

Metrics: % of measurements rejected, residual >0.3 m jumps among
accepted readings, and the longest consecutive reject streak.

**Results, `debug_inputs_20260914_161006` (1108 measurements):**

| guard | rejected | >0.3 m | max streak |
|---|---|---|---|
| none | 0.0% | 1.17% | 0 |
| last-acc (shipped) | 1.0% | 1.00% | **7** |
| last-raw | 0.3% | 1.09% | 1 |
| median3 | 0.5% | 1.00% | 2 |
| hysteres | 0.3% | 1.09% | 1 |

**Results, `debug_inputs_20260914_160801` (738 measurements):** every
design scores 0.0% rejected, 0.00% jumps, 0 streak. There is no
instability at +/-6 deg on this bag for a guard to act on.

**Interpretation.** The guard's entire measured benefit is 1.17% ->
1.00% on one bag and nothing on the other - within noise at these
sample sizes. Against that it discards data and can latch for up to 7
consecutive frames. In a pipeline that feeds a costmap, a component
that can silently withhold a person's position for ~1 s is a worse
risk than 0.17 points of range jitter.

The deeper point: the guard was added when `RAY_HALF_WIDTH_DEG` was
3.0, where instability was genuinely higher. Widening the window
addressed the cause, which made the symptom-suppressor redundant. It
was removed rather than retuned.

**If retained instead of removed,** `median3` is the best of the five:
same instability as the shipped design, half the rejections, streak
capped at 2 instead of 7.

**Caveat.** Both bags are single-person, robot parked, 1-1.6 m range.
A genuine occlusion (someone walking between robot and tracked person)
does not occur in this data, which is the case the guard was designed
for. Removal is justified by these bags; it is not proven safe for
crowded scenes, and should be revisited if multi-person data is ever
recorded.

**Reproduce:**

```
cd ~/thesis_social_navigation_ws
python3 pytest/analysis/guard.py bags/debug_inputs_20260914_161006/rosbag/*.db3
python3 pytest/analysis/guard.py bags/debug_inputs_20260914_160801/rosbag/*.db3
```

---

## Retired hypotheses

Kept deliberately — these were believed during the session and shown
wrong by measurement. Re-deriving them wastes time.

- **"The bearing residual grows with angle, so fx is wrong."** Read
  off a handful of live log lines. The binned sweep over 1108
  detections showed fx 201.308 and 287.31 are near-indistinguishable
  (sd 2.05 vs 1.96 deg), and the apparent angle dependence was an
  artefact of min-range selection, not geometry.
- **"`sweep_fx.py` measures camera/LiDAR misalignment."** It does not.
  Because the selected return is chosen by min range, `max|res|`
  equals the window width at every window setting (3/6/10/15°) — a
  saturated measurement. The script is kept for the record but its
  numbers should not be cited. The 11 Sep scan-bearing calibration
  (residual −0.1 ± 0.9°, max 1.5°) is the trustworthy measurement of
  the intrinsics.
- **"Min-range is the wrong selection rule."** Contradicted by
  Result 1 on both bags.
- **"The two weak coverage segments are range bistability."** They are
  YOLO gaps of near-identical length (Result 3).
- **"There is an unexplained YOLO dropout affecting the camera stage."**
  There is not. Camera and YOLO both ran normally; publication stopped
  at the confidence threshold (Result 4). This hypothesis also coloured
  the reading of the 14 Sep condition-C run, where detections stopped at
  t=33 s for ~100 s - that log should be re-checked for the same
  decay-into-threshold signature before being called a fault.
  RESOLVED: it does show that signature (Result 4), so the C-run
  outage was the FOV limit too.
- **"Lowering `min_publish_conf` would recover the close-range
  detections."** It would recover ~2% more detections overall and none
  of the 5.33 s gap, where YOLO produced nothing even at conf 0.15
  (Result 5).
- **"camera-ray coverage is effectively 100% given a detection."** Too
  strong. Result 3 measures 2.4 s of the node's own loss on 161006
  (17% of gap time on that bag, 15% across both).

---

## Bag inventory

| bag | duration | notes |
|---|---|---|
| `debug_inputs_20260914_161006` | 101.7 s | tuning set for Result 1; stationary + walking segments |
| `debug_inputs_20260914_160801` | 69.2 s | held-out set for Result 1 |
| `berth06_20260910_130839` | 92.4 s | 10 Sep reference, healthy pipeline rates (detector 12.7 Hz, fused 7.0 Hz, cloud 10.0 Hz) |

**Both debug bags were recorded WITHOUT `lidar_person_detector`
running.** The launch log `logs/debug_inputs_20260914_160954.log`
(which spans 161006) shows five nodes - yolo_leg_detector,
identity_fusion, person_marker_publisher, human_kf_predictor,
predicted_person_cloud - and no detector. This is why
`/lidar_person_clusters` is empty, confirmed from the launch log rather
than inferred from the empty topic.

Both debug bags contain `/turtlebot4/scan`, `/person_positions_map`,
`/lidar_person_clusters`, `/camera_ray_clusters`, the compressed camera
preview, TF and `tf_static` (2 msgs — verified non-zero, so replay
resolves `base_link`).

## Result 2 reproduce

```
cd ~/thesis_social_navigation_ws
python3 pytest/analysis/coverage.py bags/debug_inputs_20260914_160801/rosbag/*.db3
python3 pytest/analysis/coverage.py bags/debug_inputs_20260914_161006/rosbag/*.db3
```

Segment detection thresholds are `STATIONARY_WIN_S` (3.0 s) and
`STATIONARY_MAX_M` (0.25 m) at the top of `coverage.py`; coverage bin
width is `COVER_BIN_S` (0.5 s). Changing these changes the segment set,
so quote them alongside any number taken from this script.
