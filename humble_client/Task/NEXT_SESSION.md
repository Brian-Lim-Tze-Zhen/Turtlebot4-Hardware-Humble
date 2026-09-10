# Handover — TurtleBot4 socially-aware navigation

Session of 8 Sep 2026. Paste this as the first message of the next chat.

---

## 0. The one-line state

Four fusion bugs fixed and verified, MPPI horizon fixed, SocialCritic topic
name corrected. **Everything still broken traces to one root cause: the LIDAR
detector produces a cluster on the person too rarely.** Start there.

---

## 1. What was fixed today (all measured before/after on the same bag)

### `identity_fusion_node_lidar.py`

**Never-corroborated phantom.** A binding with no `last_confirmed_pos` entry was
skipped entirely by the corroboration check and survived on `LIDAR_BINDING_TIMEOUT`
(300 s) for as long as its lidar track lived — and a wall track never dies. The
comment claimed `camera_only_since` owned that case; it doesn't, that dict is only
populated for freshly *allocated* ids, not orphans handed back by
`find_orphan_to_reattach`.
Fix: anchor the corroboration check on binding time when there is no confirmation.
Measured: `calib_2m` stable id 0 held a wall cluster at 3.70 m / −14.4° for the whole
43 s bag, 437/437 `lidar_only`. After: no phantom in a 62 s live run, real detections
untouched.
Backup: `identity_fusion_node_lidar.py.bak_phantom`

**Coast drift.** `MAX_COAST_SPEED` anchored on the last *camera-confirmed* position,
so its time budget grew for the whole coast — 5 s in it permitted 7.5 m and
constrained nothing.
Fix: second check against the previous *published* position, bounding per-cycle jumps.
Measured: the −40.7° → −50.4° jump (~8 m/s) is now rejected. Partial only — 76 → 58
out-of-FOV messages, because most drift happened in ~0.5° steps that were each legal.
Backup: `.bak_coastdrift`

**Out-of-FOV bindings.** `ANGLE_GATE_DEG` bounds how far a candidate sits *from the
detection*, never where it sits in *absolute* bearing. True HFOV is 63.68° (±31.84°),
so a detection near the frame edge could bind to a cluster the camera cannot see. On
`walk_full`, stable id 0 sat at −42.7° with 5 `camera_confirmed` and never moved — so
no motion check could catch it.
Fix: `fov_ok()` applied at all three acceptance points (sticky, argmin, coast).
Parameter `fov_gate_deg`, default 35.0, ≤0 disables.
Measured: out-of-FOV 62 → 0. Side effect: identity stopped fragmenting in that bag
(one track, 175 camera_confirmed, vs two fragments) — mechanism inferred, not proven.
Backup: `.bak_fovgate`

**Stale position published as `camera_confirmed`.** `lidar_tracks[lid]` is only
refreshed when the detector republishes that id. Between republishes the cached entry
persists up to `LIDAR_TRACK_TIMEOUT` (2.0 s) and every camera detection in that window
republishes the *same frozen coordinates* tagged `camera_confirmed`.
Measured on `trial_horizon200`: lidar 95 last appeared at t24.09 at (−1.15, 0.28); the
fused output kept publishing that exact point until t26.9 — **2.8 s** — while the bbox
moved steadily across the frame. At walking pace that is ~1 m of error, most of the
clearance in a corridor.
**`camera_confirmed` does not mean the position is current.**
Fix: `LIDAR_POSITION_MAX_AGE`, parameter `lidar_position_max_age`.
Backup: `.bak_stale`

**Angle gate 20 → 12.** At 20° a static cluster and the real person could both sit
inside the gate; `sticky_lidar` then held the first pick indefinitely.
Swept on `headon_ray3`:

| gate | tracks | spread | camera_confirmed |
|------|--------|--------|------------------|
| 20°  | 2      | 0.10 m and 0.43 m | 29 |
| 12°  | 1      | 2.61 m | 20 |
| 8°   | 1      | 2.61 m | 16 |

12 keeps the continuous track with the most confirmations. **Not verified against
tape-measure ground truth, and swept on ONE 7.5 s encounter** — re-run on a second bag
before treating as final. Backup: `.bak_gate20`

### `lidar_person_detector.py`

**Ray-consistency map veto.** `static_inflation_cells` went 2 → 0 because k=2 deleted a
person near furniture; that removed the only suppression of wall fragments. Ray
consistency casts from the sensor toward the candidate and finds the map's own first
occupied cell along that ray — a candidate closer than it by more than
`ray_clear_margin` is something in *front* of the obstacle.
Parameters: `use_ray_consistency` (default true), `ray_clear_margin` (0.25),
`ray_max_range` (8.0). Set `use_ray_consistency:=false` to restore cell lookup.
Measured on `calib_2m`: clusters 988 → 138, near-obstacle 82% → 1%, person detected on
essentially every scan t10–25 at 2–3 m forward.
Backup: `.bak_raycheck`

### Nav2 config

**MPPI horizon.** `time_steps` 120 → 200, `batch_size` 2000 → 1200. Rollout reach =
`time_steps × model_dt × vx_max` = 1.86 m → 3.10 m. Below that reach MPPI could not
see a person until they were nearly on top of it, and reversing was the only remaining
trajectory.
Measured (`trial_social_off` → `trial_horizon200`): reversals 13% → 1%, hard turns
(|ω|>1.0) 19% → 7%, control-loop misses 18 → 9. **Improved on both metrics at once.**

**SocialCritic topic name.** Was `/fused_person_positions`; the fusion node publishes
`/person_positions_fused`. The critic received nothing and logged "no person data this
cycle — critic is inert" continuously.
**Every earlier ablation compared inert against inert. Those runs are void.**

### Tooling

`analysis/bagreader.py` — dispatches on extension so the scripts read `.db3` as well as
mcap. `tfutil.py` line 3 now imports from it (`.bak` kept).

Replay method, used all day: `ROS_DOMAIN_ID=77` to isolate from the live robot,
`use_sim_time:=true` on every node, `--clock 100` on playback. **Always 1× — the
staleness and coast checks are wall-clock driven, so `--rate` corrupts the result.**

---

## 2. The root cause — read before changing anything

**The detector produces a cluster on the person on only a fraction of scans.**

Evidence from today:
- The stale cache exists because bindings go unrefreshed for seconds at a time.
- Identity fragmentation happens when a stale position drifts far enough from fresh
  evidence that the matcher allocates a new id instead of continuing the old. On
  `trial_social_on`, three ids in one second (t22.74 / t23.17 / t23.47), with sid 2 and
  sid 3 *overlapping* — MPPI was told two people were present.
- The staleness gate can't be used yet because it exposes the underlying rate:

| gate | fused rate | longest freeze |
|------|-----------|----------------|
| none | 6.4 Hz | 2.42 s |
| 0.5 s | 2.5 Hz (replay) / **0.7 Hz (live)** | 0.41 s |
| 0.3 s | 1.3–1.9 Hz | 0–2.7 s (metric unreliable at this rate) |

At 0.7 Hz live the avoidance got *worse* (reversals 1% → 2%, hard turns 7% → 13%) —
the person cloud was starved.

This is the same sparsity the previous handover opened with. Everything circles back.

---

## 3. Tasks — in order

### A. Detection-range curve (C1) — do this first
For each calibration distance: how many returns the legs produce, cluster width, how
many survive `min_points_per_cluster`, the width filter, the map veto, the motion
filter. Output a stage-by-stage survival table vs range. Bags are ready and have
tape-measure ground truth.

### B. Set `min_points_per_cluster` from that curve, not arithmetic
Candidate 4 → 3. At 5 m one leg subtends 1.15° ≈ 3.4 returns at 0.333° resolution.

### C. Re-decide the staleness gate once the rate is up
The gate is *correct* — it stops confidently-wrong positions reaching MPPI. It is
currently unusable only because the detector can't feed it.

### D. Re-run the angle-gate sweep on a second bag
12° is from one 7.5 s encounter. `fov_live` has a longer, cleaner sequence.

### E. Verify fused position against the tape measure (C4)
Still not done. Every "improvement" measured today is a *proxy* (spread went up,
plateaus went away) — none is accuracy. Movement is not correctness.

---

## 4. Known issues, not fixed

- **Bbox clipped at the top** — every YOLO detection has `y1=0..2`. Height
  underestimates the person, so `bbox_range` *over*estimates distance, biasing
  `RANGE_RATIO_GATE` and `RANGE_COST_WEIGHT`. **Root fix is mounting geometry: tilt the
  camera up.** Disabling `RANGE_COST_WEIGHT` changed 1 message in 35, so the range term
  is *not* currently the dominant error — but the bias is real and the gates were tuned
  against it (circular, as the previous handover warned).
- **ByteTrack fragmentation** — 3–4 camera ids for one person in a few seconds.
  Upstream of the fusion node entirely.
- **KF holds a dead identity ~1.3–1.7 s** after fused output stops, publishing a frozen
  prediction into the costmap.
- **Constant global rerouting** — the predicted person cloud is in the *global*
  costmap, and the BT's `ComputePathToPose` RateController was raised 1 → 5 Hz. A
  dynamic obstacle in a costmap that replans at ~2.3 Hz makes the global path swing
  continuously; `PathAlignCritic`/`PathFollowCritic` then drag the controller after it.
  Design tension: either drop the replan rate, or take people out of the global costmap
  and let the SocialCritic own them.
- **Odom QoS** — `bt_navigator` declares `odom_topic` **twice** (`odom_reliable` then
  `odom`); the second wins. Still on BEST_EFFORT.
- **Map completeness ceiling** — 44.7% of scan returns land on cells `lab2_map` marks
  occupied, and the offset sweep peaks at 0.00 m, so alignment is *correct* and the map
  is simply missing over half of what the LIDAR sees. The desk that captured a track had
  only 15/169 occupied cells within 0.3 m; the one correctly rejected in `calib_2m` had
  45/169. **No map veto can reject an obstacle the map never recorded.** State this as a
  limitation.

---

## 5. First valid SocialCritic ablation

`trial_horizon200` (off) vs `trial_social_on` (on), identical except the critic:

| | reversals | hard turns | mean \|ω\| | fused rate |
|--|-----------|-----------|-----------|------------|
| off | 1% | 7% | 0.344 | 6.4 Hz |
| on  | 4% | 2% | 0.273 | 7.9 Hz |

Smoother turning, more reversing. Reading: the social critic makes close passes
expensive, and in a 1.5 m corridor no cheap lateral trajectory exists, so backing off
becomes lowest-cost. Arguably correct yielding behaviour, but not the early detour
wanted.
**Caveats: encounters were not identical (9.2 s vs 13.3 s of tracking), and 27 "critic
is inert" warnings mean it lacked data part of the time.** `social_distance` is still
0.94 m — in a 1.5 m hallway, 2.0 would blanket the whole corridor; ~1.2 is the
realistic ceiling.

---

## 6. Data

Calibration (robot stationary, tape marks, **measured front-edge-to-toes**):
`calib_2m`, `calib_3m5`, `calib_5m`, `calib_empty` (false-positive baseline — 0
clusters, 0 detections, 0 fused).

**Range offset ≈ 0.46 m, constant at 2 m and 3.5 m.** LIDAR sits 40 mm *behind*
base_link, so front-edge-to-LIDAR is 0.209 m; remainder is toes-to-shin. Not growing
with range, so no scale error and no TF fault. Subtract a constant.

Live trials: `trial_social_off`, `trial_horizon200`, `trial_social_on`,
`trial_stale05_live`, `fov_live`, `phantom_test`, `headon_ray2`, `headon_ray3`.
Replays: `stale03`, `stale03b`, `stale05`, `gate8`, `gate12`, `rcw0`, `ray_*`.

`walk_full` has `cmd_vel` = 0 — the robot never moved. Perception data only, not a
navigation trial.

---

## 7. Standing instructions

- **Establish ground truth before analysing.** Every metric used today was a proxy.
  "Spread went up" measures motion, not accuracy.
- **Check what a metric measures before reporting it.** The plateau detector cannot
  distinguish a frozen cache from a person standing still at low publish rates. The
  ±15° "closest forward return" hides a person behind any nearer object in the arc.
- **Note the measurement convention in the bag notes.** Distance from *what* to *what*.
  A whole test was wasted on "10 cm" meaning desk-to-person, not robot-to-person.
- **One change at a time, measured on the same bag before and after.**
- **Replay at 1×.** Rate-dependent checks make any other speed meaningless.
- Reference poses: `START x=5.123 y=3.617 qz=0.954 qw=0.300` ·
  `GOAL x=-0.974 y=7.088 qz=-0.275 qw=0.961`
