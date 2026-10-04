<!-- Append below the last Result in RESULTS.md, above "Retired hypotheses".
     Retired-hypothesis bullets and bag rows go into their existing sections. -->

## Correction to Results 1, 2A, 3 and 6 — rays were cast 90 deg off

**Found 15 Sep; see Result 7.** Until the mount-yaw fix,
`camera_ray_person_node.py` searched the raw scan array (frame
`rplidar_link`) with bearings computed in `base_link`. The RPLIDAR is
mounted at +90 deg yaw, so every ray was cast to the robot's side.

| Result | Status |
|---|---|
| 1 (ray rule, +/-6 deg window) | **invalid** — ranges came from a side wall; the very stable 1.29–1.30 m readings fit a wall, not a person |
| 6 (guard removal) | **invalid** for the same reason; see also Result 11, which reintroduces range tracking with real occlusion data |
| 2A (stationary coverage), 3 (gap attribution) | **re-check** — message counts are unaffected, positions were wrong |
| 4, 5 (YOLO publish threshold) | unaffected — camera only |

Re-run 1, 2A, 3, 6 on bags recorded after 16 Sep.

---

## Result 7 — LiDAR mount yaw: bearing frame mismatch

**Claim.** The camera-ray node cast every ray 90 deg to the side of the
intended bearing. Fixed by converting the bearing into `rplidar_link`
using the mount yaw read from TF at startup.

**Evidence.**
- `tf2_echo base_link rplidar_link` → RPY yaw **90.000 deg**.
- Person standing at the left frame edge: logged bearing +32 deg, hit
  +26 deg; after fx 201.308 → 287.31, bearing +24.7 deg, hit +19.0 deg.
  The hit sat at **bearing − 6.0 deg in every line**, i.e. pinned to the
  window edge regardless of fx.
- Raw scan over the searched window (array −5 to +33 deg): one smooth
  surface at **1.55–1.70 m**, no leg cluster — a wall.
- After the fix: hits fall inside the window at the person's real range.

**Consequence.** "fx 287.31 fixes edge rays" was not the cause (see
Retired hypotheses). fx 287.31 is kept (matches the 11 Sep calibration)
but its effect has not been isolated from this fix.

---

## Result 8 — Camera-primary identity: fragmentation vs the old fusion node

**Claim.** Carrying identity by map-position continuity, with LiDAR
never creating or anchoring a person, removes most identity
fragmentation and the lidar-only phantoms of `identity_fusion_node_lidar.py`.

**Old pipeline, live (15 Sep, robot parked).**

| run | scenario | people | camera tracks | LiDAR track ids | stable ids |
|---|---|---|---|---|---|
| A | stationary 15.7 s | 1 | 1 | mild churn | 1 |
| B | lateral ×3, 44 s | 1 | 7 | ~33 (≈1 per 1.3 s) | **3** |
| C | head-on ×3, 36 s | 2 | 2–3 | ~36 | 2 + 1 lidar-only phantom at 8.59 m |

LiDAR churn under motion was the same for lateral and head-on (B ≈ C),
so "radial motion churns less" is retired. Fragmentation required
camera loss **and** churn together: C held one id through 15 LiDAR
drops because the camera never lost the person.

**New identity node, replay of the same bags** (note: node output was
still pre-Result-7, so identity logic is tested, positions are not).

| bag | old (live) | new (replay) |
|---|---|---|
| A | 1 | **1** |
| B | 3 | **1** (8 camera switches, all reattached at 0.00–0.30 m) |
| C | 2 + phantom | **2**, no phantom (people 0.36 m apart kept separate) |

**Two failure modes found live and fixed (proto_F, one person
standing, robot moving).**

| version | stable ids |
|---|---|
| original (time-based "live holder", 1.0 s) | 4 |
| + PENDING 0.4 s | 3 |
| + same-frame rule (30 ms) | **2** (second id after a 20 s camera gap — correct) |

- *Switch refused:* ByteTrack switch arrived **0.17 s** after the old
  id's last frame (cam 1 last 965.973, cam 2 first 966.145), inside the
  liveness timeout.
- *Flip-flop taken for a second object:* ByteTrack alternated ids in
  blocks (3,3,3, 2×7, 3), never in the same frame. Two real objects
  appear in one frame; YOLO publishes a frame's boxes **~1.2 ms** apart
  (270.0414 / 270.0426). Rule: second object only if seen within 30 ms.

**Limits.** Replay YOLO is not frame-exact (Result 5). The same-frame
rule has only been checked in simulation and on proto_C; a live
two-person run (crossing) is still missing.

---

## Result 9 — Moving robot: stationary person's cloud stretched

**Claim.** The stretch had two causes: KF velocity from jitter, and
(dominant) detections paired with a newer scan/pose than the image.
Fixes: KF stillness gate, and time-matched scans.

**Measured detection delay** (image stamp → YOLO publish): **89–111 ms**
(mode ~100 ms). PC and robot clocks consistent.

**Before (proto_H), id 0, person standing:**

| displacement per 1 s | stretch of 1 s prediction |
|---|---|
| ≤ 0.16 m | ≤ 0.18 m |
| 0.18–0.27 m | 0–0.68 m |
| **0.45–1.47 m** | **0.29–1.36 m** |

**After (proto_H2): image stamp in `/person_positions_map` field [7],
nearest buffered scan, TF at scan stamp, `STILL_MAX_DISP_M` 0.30.**

- Displacement per 1 s while standing: **≤ 0.18 m**, stretch 0.00 m.
- All predictions: ≤5 cm 131, 5–20 cm 21, 20–50 cm 8, >50 cm 49.
  **All 49 >50 cm fall in the first ~4 s after an id was created**
  (136–140 s id 0; 167–168 s id 1). None while standing.
- `scan_dt` −87 to −116 ms; 2 of the logged lines had no scan within
  150 ms (closest −160 ms). The buffer never holds a scan newer than
  the image, so a ~100 ms residual offset remains; not needed for this
  scenario.

**Offline KF check (proto_F, stillness gate 0.15 m):** 219 samples at
vel_filt (0.00, 0.00); the remaining ones ≤0.2 m/s, largest burst
coinciding with a 0.25 m ray jump.

**Limits.** Start-of-track stretch is the 1 s gate warm-up (or real
walking in); not separated.

---

## Result 10 — Stale obstacle marks in open space

**Claim.** Leg marks stayed in both costmaps after a person left because
open-space rays return `inf` and were discarded. Fixed in config.

**Evidence.** One scan: **173 `inf`/`nan` rays**. `inf_is_valid` was
absent (default false) in local and global `scan` sources. Adding it
cleared the global costmap; the local `VoxelLayer` still kept marks
(clear-service test: marks did not return → stale, not real) until it
was replaced by a 2D `ObstacleLayer`. Remaining marks only in cells
beside wall edges, where rays hit the wall first; they clear once
viewed from another angle.

**Scope.** Applied to `social_nav2_ablation_E_critweight20_socialcritic_on.yaml`
only. The other three configs must get the same change before any
ablation comparison.

---

## Result 11 — Legs hidden or mis-ranged: range tracking for camera rays

**Claim.** Taking the nearest LiDAR return in the ray window fails when an
unmapped object hides the legs, when a person first appears at the image
edge, and when the robot turns. Per-track range tracking with a relative
bbox-width release, an edge-wait lock, short id-switch inheritance and
odometry yaw compensation fixes the lateral, occlusion, walk-away and
moving-robot cases; a wrong first lock on a standing person remains.

**Not a sensor limit.** Unobstructed, the scan resolves both legs at
**2.61–2.67 m** (−4 to +2 deg) with 10 m returns between them.

### A. Legs hidden by an unmapped object (proto_G, lateral ×3)

Live: 4 stable ids, every id's last position on the door; ray cam 8
entry 2.16/2.45 m → mid-crossing 5.46/5.49 m.

| version (replay) | near ≤4 m | far >4 m |
|---|---|---|
| live: reset to nearest after 3 misses | 21 | 29 |
| reset only to a nearer return | 21 | 25 |
| + `MAX_RANGE_SPEED` 2.0 → 0.5 m/s | 20 | 27 |
| + timeout on camera silence, gate cap 0.6 m | 23 | **0** |
| + relative bbox-width release | 22 | **0** |

Refusals did not refresh the old 1 s track timer, so hidden legs silently
reset the track to the door (row 3). Row 2 crept outward: 2.44 → 3.22 →
4.59 m → door.

### B. Person walking away (proto_C)

Row 4 locked out a retreating person (tracking 2.17 m, refused 5.29 →
6.22 → 6.78 → 7.38 m, max miss 29). Release when a return matches the
range implied by bbox width for 3 frames (±0.5 m): max miss **17**,
releases 2.17 → 6.48 m (bbox implied 6.18 m), 4.30 → 5.46 m (5.40 m).

### C. Wrong first lock (proto_K2) — which width rule

cam 5 locked 2.89 m on its first box (x1 = 1), then refused the legs at
3.90–3.92 m for 138 frames.

| release rule | G far | K2 | C |
|---|---|---|---|
| relative (last_r × w0 / w) | **0** | stuck | **released, miss 17** |
| absolute (185 px·m / w) | 28 | released 2.89 → 3.91 | – |
| absolute + continuity gate | 18 | released | – |

- range × width on 5 live logs: median 175–215 px·m, p10 66–142, p90
  212–265 — too noisy as a range.
- A box shrinking behind an occluder looks like a person walking away
  (G: released 2.44 → 5.44 m, bbox implied 5.44 m = the door).
- The relative rule cannot fix K2: its width reference was taken while
  locked on the wrong range (w0 = 36 px at 2.89 m).
- **Decision: relative rule.** K2 is a documented limit.

Edge boxes: x1 = 1, 4, 5, 6, 9 were still truncated (28 px vs ~47 px at
3.9 m), so boxes within **15 px** of an edge give no width reference.

### D. Object placed in front while tracking (live proto_K3)

Two routes captured the range: the nearer-only reset (removed) and every
new ByteTrack id starting fresh (new id inherits a range track seen
within `INHERIT_S` at ±8 deg bearing).

- Pass rate **456 / 545 = 84 %** (K2: 75 %).
- Box in front: range 3.04 → 2.70–2.77 m (≤0.33 m toward the robot,
  inside the 0.6 m cap), no background hits.
- First lock 0.60 m (the box) recovered in ~5 frames.
- Walk back: smooth 3.6 → 7.85 m; release 6.70 → 7.36 m (bbox 6.93 m).
- One identity switch: YOLO split the person into id 22 (98,0,147,76)
  fully inside id 20 (92,0,157,153); IoU 0.37 < 0.45 dedup. Fixed in the
  YOLO node: also drop a box ≥85 % contained in a kept box.

Live proto_G2 (lateral ×3, after all fixes): **1 stable id, 0 far hits,
208/208 published, 0 "second object"**. No refusals, so the object did
not hide the legs in this run.

### E. Moving robot (Nav2 walk-past, proto_L / L2)

- Scan lag while driving: −84 to −195 ms; limit raised 0.15 → 0.30 s
  after live drops ("closest −153 to −190 ms").
- Person's box swept ~110 px in 0.5 s (~45 deg/s): ~8 deg of rotation
  inside the lag, more than the ±6 deg window. **Odometry yaw
  compensation** (buffered odom, interpolated/extrapolated ≤0.3 s)
  applied corrections of −3.8 to +6.0 deg. First TF-based version
  returned 0.0 deg (lookup failed silently); first odom version got no
  messages (RELIABLE vs BEST_EFFORT QoS).
- **Edge-wait lock:** proto_L cam 12's first box (206–247 px) locked
  background at 7.59 m, then refused the approaching person 6.50 → 2.34 m
  (38 refusals). No lock while the box is within 15 px of an edge
  (≤3 s). Replay: G far 0, C miss 17 unchanged; K2 unchanged (a real
  object at 2.89 m was nearest on the first inside box).
- proto_L2 live: 46/105 published (44 %), 3 stable ids. Far rays
  (5.9–7.5 m) were the person (box 27–34 px wide; person started far).
  Largest loss: cam 13 inherited 3.11 m from an id seen 0.87 s earlier
  while the person was at 6.5 m → ~3.6 s refused.
  `INHERIT_S` 1.0 → 0.3 s (replay): stale inherit gone, cam 13 publishes
  6.83/7.46 m, refusals 7 → 4. **Live test pending.**

### Known limits
- Wrong first lock on a standing person (K2) persists until the camera
  loses them.
- An occluder ≤0.6 m in front can pull the position toward the robot by
  up to 0.6 m.
- Under robot motion YOLO/ByteTrack churns: 9 track ids and 72 REJECT
  lines (mostly id:-1) in a 10 s crossing (L2).
- At 6–7 m, a 1.5 m range jump between two camera ids split one person
  into two stable ids (L2).
- OAK-D depth along the ray would remove the dependence on leg
  visibility, but the depth pipeline is disabled on this robot because it
  drains the battery too fast for experiment-length runs.

### Retired hypotheses (add to existing section)

- **"fx 287.31 fixes rays pinned to the window edge."** The pinning was
  the 90 deg mount yaw (Result 7).
- **"Holder id publishing again means a second object."** ByteTrack
  revives lost ids for the same person; only same-frame coexistence
  separates two objects (Result 8).
- **"Radial (head-on) motion churns LiDAR tracks less than lateral."**
  B and C churned equally (Result 8).
- **"Cloud stretch while the robot moves is the stillness threshold."**
  The large cases were 0.7–1.5 m ray jumps from scan/image timing;
  threshold tuning alone could not fix them (Result 9).
- **"A 2D LiDAR cannot see a person straight ahead; switch to 3D."**
  The scan resolves both legs at 2.6 m when unobstructed (Result 11).
- **"bbox width can estimate range well enough to pick a return."**
  range×width p10–p90 spans roughly −50 % to +30 %; and an occluder shrinks
  the box like a retreating person (Result 11 C).
- **"Rays > 5 m during the Nav2 walk-past are background."** The person
  started far; box width put them at 5.4–6.9 m (Result 11 E).

### Bag inventory (add rows)

| bag | content |
|---|---|
| proto_A_stationary | old pipeline live, person still 15.7 s |
| proto_B_lateral | old pipeline live, lateral ×3 |
| proto_C_headon | old pipeline live, head-on ×3, 2 people in view |
| proto_F_robotmoving_personstill | camera-ray live, robot moving, before stillness gate |
| proto_H_robotmove_still | camera-ray live, Nav2 goal, before scan time-matching; `/turtlebot4/plan` |
| proto_H2_robotmove_still_stampfix | same scenario after time-matching |
| proto_G_lateral | camera-ray live, lateral ×3, legs occluded by unmapped object |
| proto_K2_occlusion | camera-ray live, box in front; wrong first lock from edge entry |
| proto_K3_occlusion | camera-ray live after range fixes; 84 %, split YOLO detection |
| proto_G2_lateral | lateral ×3 after all fixes; 1 id, 208/208 |
| proto_L_nav_walkpast | Nav2 walk-past before yaw compensation |
| proto_L2_nav_walkpast | Nav2 walk-past with yaw compensation + edge-wait; 44 % |
