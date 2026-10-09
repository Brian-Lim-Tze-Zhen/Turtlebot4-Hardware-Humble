# Hardware day 9 Oct 2026 — rec15 on the robot

Config `workspace/config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_hw.yaml` (critic `SocialLaneHoldCritic`, lib md5 14ee9bdd),
map `hall_map_name.yaml`, perception `perception_camray_lane_noghost_bringup.launch.py`, zone stack as on 8 Oct,
`blocked_person_beep_node_front.py hold:=true`, `beep_retry_node.py`. Nav2 log at start: "side hold: on ... [SocialLaneHoldCritic]"
and "pass side measured against the robot-goal line (goal from goal_pose)".
Start (-6.80, 13.50) heading 141°, goal (-12.20, 17.75). Laptop clock 3 ms off NTP.

- Before the runs (~10:57): scan delay median 0.117 s, p95 0.17 s, one 1.6 s burst in 8 s; odom median 0.012 s. No robot resync.
- Robot sent to start: reached (-6.92, 13.53), heading ~134°.

## Runs

| Time | Bag | Result | Brian |
|---|---|---|---|
| 11:04 | `headon_rec15_run01_20261009_090432` | **Wi-Fi blackout at the start.** Walker stood at (-12.5, 18.0), id 1, camera confirmed, 7.1 m. Last person detection t+0.46 s; then no scan for 1.37 s (t+0.3 to t+1.7, more gaps of 0.3–0.6 s up to t+3.2), no camera-ray cluster for 2.9 s (t+0.46 to t+3.33); cmd_vel at ~4 Hz instead of ~20 Hz from t+0.5 to t+2.5; controller "Transform data too old map->odom" t+0.7 to t+2.5. Walker re-detected as id 2 at 3.3 m (t+3.4). Lane rule at t+4.2 with the walker **2.84 m** ahead (sim: 6–7 m), **robot keeps LEFT** (walker's mean offset -0.29 m, fixed t+4.8). Closest t+6.2 s: centre 0.75 m, clearance 0.56 m, robot vx 0.21 m/s. Max lateral deviation 0.51 m, no stops, goal reached (26.1 s of motion). Occlusion slow-down t+5.9 to t+8.2. | Robot did not avoid smoothly. |
| 11:11 | `headon_rec15_run02_20261009_091048` | Link fine (scan gaps 0.37 s, 0.31 s). Start heading ~153° (12° left of the hallway). Walker id 4 camera confirmed at 7.5 m. **Lane rule at t+1.8 s with the walker 6.47 m ahead, robot keeps RIGHT** (offset +0.07 m, fixed t+2.5) — timing as in simulation. Global plan moved to the right lane at once (-0.86 m only ~1.4 m ahead of the robot). Controller then commanded **wz -1.16 / -1.07 rad/s** (t+2.0 to t+3.5): the robot turned up to **95° right of the hallway** while moving only 0.38 m sideways (t+5.0), then +1.24 rad/s back, yaw swinging -46/-29/-41° until t+8.5. The turn took the walker out of the camera view (YOLO box slid to the left edge t+2.5 to t+2.8, half FOV ~23°); LiDAR-only track drifted to +1.1..+1.8 m lateral (not the walker), predictor coasted to t+5.1, then **no track for the pass**; "no person data, critic inert" at t+11.1. Nearest LiDAR point 0.62 m at t+7.4 (bearing -30°, may be wall). Max lateral deviation 0.91 m, no stops. Collision monitor passes about half of the smoothed commands, lag 0.09-0.21 s (not the cause). Camera-ray range lock 7.34 m vs 5.78 m measured at t+2.0, released t+2.2 (KF speed jumped to ~1.0 m/s). | Robot did not avoid smoothly. |
| 11:15 | `headon_rec15_run03_20261009_091505` | Walker id 6 camera confirmed at 7.1 m. **Lane rule at t+2.0 s with the walker 6.32 m ahead, robot keeps RIGHT** (offset -0.06 m, fixed t+2.7). Same pattern as run 2: hard right turn (max heading 73° off the hallway, wz up to 1.31 rad/s), walker LiDAR-only from t+3 s, no track after t+4 s; "critic inert" from t+11.8. Nearest LiDAR point 0.61 m at t+8.4 (bearing -30°). Max lateral deviation 0.93 m, no stops. Analysed afterwards (not straight after the run). | |
| 11:27 | `headon_rec15_run04_20261009_092705` | CAMERA=1 (44 MB). Walker id 9 camera confirmed at 7.3 m. **Lane rule at t+1.8 s, 6.50 m ahead, robot keeps LEFT** (offset -0.17 m > 0.15, fixed t+2.4). Hard left turn (yaw up to +98° off the hallway at t+4.0-4.5), walker LiDAR-only from t+2.85 (KF rotation-gated, velocity frozen at 0.67 m/s), predictor pruned the track at t+5.5. Real walker went by about t+5.5-7.5 (nearest scan 0.54 m at t+5.5, then sweeping to the side). **Slow-down after the pass is a ghost**: the critic coasted the last estimate (max_coast_time 6.0 s, coast_weight 1.0) at the frozen 0.67 m/s, so the ghost reached the robot only at t+9.4; occlusion slow-down ON t+9.4, scale 0.49 -> 0.30 -> 0.11, robot 0.06-0.13 m/s t+9.5 to t+11.5 (controller command, not the collision monitor), coast expired t+11.1 (critic inert t+11.4), back to 0.30 m/s at t+12. Max lateral deviation 0.94 m, no stops, goal reached (28.8 s). | Asked: why did it slow down after passing. |

- ~11:10 after run 1: 20 s probe, **no scan at all**, odom delay median 0.53 s, max 3.4 s. Link not usable for a run.

## Turn sharpness: robot vs simulation (9 Oct, analysed after run 3)

Heading from 0.10 m displacement steps, start-to-goal leg; sim from `trajectory.csv` (ground truth), robot from odom in map (map->odom TF).
Scripts: scratchpad `simhead.py` / `robhead.py` (session 9 Oct; not yet in the repo).

| Runs | max heading off the corridor | max wz (rad/s) | time with |wz| > 1 rad/s |
|---|---|---|---|
| sim `headon_rec15_trial01..10` (10) | 60–70° | 0.84–1.16 | 0.0–0.3 s |
| robot rec15 run01–03 (today) | 77°, 91°, 73° | 1.18–1.31 | 0.2–0.8 s |
| robot v4noghost(tf05) 8 Oct (5) | 71–98° | 1.24–1.99 | 0.5–1.5 s |

The sharp swerve is already in the tuning (simulation turns 60–70° too); the robot turns 10–30° further and faster.
Robot-only effect: the turn takes the walker out of the camera view (half FOV ~23°) and the LiDAR-only track does not hold the walker, so the pass is made without a track (runs 2, 3).

## Weaving (Brian: "robot wiggles"), analysed ~11:45

Turn-direction flips (|wz| > 0.15 rad/s, sign change, while vx > 0.02 m/s), odom wz; scratchpad `wiggle.py`.

| Runs | flips per run | per metre |
|---|---|---|
| sim `headon_rec15_trial01..10` (odom wz in trajectory.csv) | 3–9 | 0.3–0.8 |
| robot rec15 run01–04 today | 15–23 | 2.1–3.3 |
| robot v4noghost(tf05) 8 Oct | 14–24 | 2.0–3.4 |
| robot lanerule 5 Oct | 22–31 | 3.2–4.5 |

- Commanded, not sensor noise: cmd_vel_nav flips 19–35 per run (run01–04).
- Not the social critic: in run 4 the weaving goes on from t+12 to t+28 s with "no person data, critic inert".
- AMCL map->odom corrections in run 4: 26 jumps of 2–10 cm / up to 1.9° in 29 s; some flips follow a jump within ~0.3 s, many do not. Not shown to be the cause.
- Open. Candidates: AMCL jumps, late odom/TF (state delay), controller rate (one "missed its desired rate" in run 4).

## Noghost critic, offline replay (9 Oct ~11:50)

New package `workspace/src/social_critic_lane_noghost` (class SocialLaneNoGhostCritic, lib md5 5551fdef) = social_critic_lane_hold
(unchanged, 14ee9bdd) + `occlusion_slow_ignore_coasted` + `coast_ghost_radius`. Config
`config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_noghost_hw.yaml` = rec15 + those two (true, 1.0).
Replay: `bags/replay_rec15_port/tools/replay_all_9oct.sh` (docker --network none, domain 77), out in
`bags/replay_rec15_port/out/humble_{rec15,noghost}_9oct/`. Open loop: robot path and tracks come from the bag.

| Run | rec15 replay | noghost replay |
|---|---|---|
| run02 | RIGHT at 6.46 m; occlusion slow-down ON lane+7.9 s | RIGHT at 6.46 m; no slow-down |
| run03 | RIGHT at 6.33 m; slow-down ON lane+5.9 s, off +2.4 s | RIGHT at 6.33 m; no slow-down |
| run04 | LEFT at 6.50 m; slow-down ON lane+7.7 s | LEFT at 6.50 m; no slow-down; "track 9 dropped as a ghost (0.97 m)" lane+7.9 s |

Side decisions and their timing unchanged. Not yet run on the robot or in simulation.

## Noghost on the robot (9 Oct, from ~11:47)

Nav2 restarted with `config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_noghost_hw.yaml` (first try failed: the launch
shell had not sourced the new package -> controller_server FATAL "SocialLaneNoGhostCritic does not exist", bringup aborted,
no costmaps in RViz; fixed by `source install/setup.bash`). AMCL restarted with it; pose re-set in RViz.
Note: the bag's `md5sums_critic_lib.txt` (record_hw_run_v4.sh) does not list libsocial_critic_lane_noghost.so (5551fdef); the config in the bag names the critic.

| Time | Bag | Result | Brian |
|---|---|---|---|
| 11:48 | `headon_rec15noghost_run01_20261009_094820` | Walker id 1 camera confirmed at 7.1 m. **Lane rule at t+2.0 s, 6.28 m ahead, robot keeps LEFT** (offset -0.32 m, fixed t+2.6). Hard left turn again (yaw up to +77°), walker LiDAR-only from t+2 s. **"track 1 dropped as a ghost (coasted to 1.00 m from the robot)" at t+6.1**; no occlusion slow-down; robot 0.29-0.31 m/s from t+6 to t+24 (no crawl after the pass). Real walker going by about t+5.5-7 (nearest scan 0.77 m at t+5.5, then sweeping to the side) - **the ghost was dropped while the real walker was close**; the robot was already 0.8-0.96 m to the side, so no effect here, but the rule removed the critic's last estimate during the pass. Weaving after the pass continues (yaw -19 .. -3°). Max lateral deviation 0.92 m, no stops, goal reached (27.0 s). | |
| 11:54 | `headon_rec15noghost_run02_20261009_095414` | **Lane rule at t+2.2 s, 6.44 m ahead, robot keeps LEFT** (offset -0.21 m, fixed t+2.8). Brian: passed on his right-hand side again. Not analysed further than the side (see below). | Robot passed on my right both times. |
| 12:03 | `headon_rec15noghost_run03_20261009_100317` | Localization checked and fixed before the run (heading 3.75° -> 0.25°). Walker id 10 standing at 7.3 m until ~t+1.5 s. **Lane rule late: t+3.1 s, 4.54 m ahead** (sustained-approach rule; camera range 7.43 -> 5.50 m in 1 s), **robot keeps RIGHT** (offset **-0.00 m**, fixed t+3.7). Short, hard swerve: yaw down to -91° (facing the right wall) t+5.5, moving sideways l -0.16 -> -0.58 m with s almost constant (1.18 -> 1.27 m) t+4.5 to t+6.5. Walker LiDAR-only from t+4.3 (rotation gate on/off); real walker going by ~t+6-7 (nearest scan 0.57 m at t+6.5, bearing +67°, robot facing the wall). **Ghost dropped at t+6.4 at 0.38 m** - here the coasted target was the real walker going by (it only became 'coasted' at 0.38 m). No slow-down after the pass, 0.29-0.31 m/s from t+7. **Stop t+24.0-24.9 near the goal: Wi-Fi burst** (scan 1.2-1.4 s old, collision monitor ignoring it; controller 'extrapolation into the future', follow_path aborted 4x, BT re-sent) - not person-related. Max lateral deviation 0.72 m, goal reached (26.5 s). | |
| 12:09 | `headon_rec15noghost_run04_20261009_100916` | **Start stall t+0.3-1.3 s (Wi-Fi/TF)**: controller 'extrapolation into the future', follow_path aborted 5x, BT re-sent; robot 0 m/s until t+1.3 (the 1 stop >= 0.5 s). Walker id 13 camera confirmed at 6.8 m, walking from ~t+2. **Lane rule t+3.2 s, 5.82 m ahead, robot keeps RIGHT** (offset **+0.00 m**, fixed t+3.8). Swerve again hard: yaw -97° at t+5.5, sliding sideways (l +0.14 -> -0.41) with s almost constant (0.66 -> 0.77 m) t+4.0 to t+6.5; lateral down to -0.74 m at t+8.5. Walker LiDAR-only from t+4; ghost dropped at 0.98 m t+7.6. Nearest LiDAR point ahead 0.56 m at t+7.9, bearing -30° with the robot at yaw -31..-56°, i.e. towards the right-hand side (probably the right wall). No slow-down after the pass (0.29-0.31 m/s). Max lateral deviation 0.89 m, goal reached (29.3 s). Localization at the far end after the run: heading -0.75°, OK. | |
| 12:13 | `headon_rec15noghost_run05_20261009_101253` | **No camera recording from this run on** (Brian: camera recording loads the Wi-Fi), 20 MB. No controller aborts. Localization at the start -1.0° (fit), OK. Walker id 17 at 7.3-7.7 m; camera range held at 7.30 m then 'motion-release 7.30 -> 5.64 m' at t+2.2 -> KF speed 1.5-1.8 m/s. **Lane rule t+2.4 s, 5.62 m ahead, keeps RIGHT; fixed RIGHT with offset -0.22 m** (beyond -0.15, but the side hold keeps the first side unless > 0.30 m). Swerve -94° at t+4.5, sliding sideways (l -0.11 -> -0.61, s 0.92 -> 1.01) t+3.5 to t+6.0. LiDAR-only from t+3.0; KF velocity frozen by the rotation gate at (1.65, -0.57), so the **predicted walker drifted to the robot's right** (l -0.42 at t+4.0, -0.79 at t+4.7, -1.02 at t+5.1) into the robot's side; predictor pruned it at t+5.7. **Occlusion slow-down ON t+5.5-6.9** (a published, not coasted, target) -> 0.07-0.08 m/s t+6.0-6.5 during the pass. Real walker going by ~t+6 (nearest scan 0.47 m). No stops, max lateral deviation 0.69 m, goal reached (26.6 s). | |
| 12:21 | `headon_rec15noghost_run06_20261009_102112` | No camera. Localization checked by Brian before the run; bag fit +0.5°. **Lane rule t+1.7 s, 6.85 m ahead, robot keeps LEFT** (again at t+2.2, 6.49 m), **fixed LEFT at t+2.3, walker's mean offset -0.19 m** (critic window); whole t+0.5-3.5: fused -0.13 m, camera rays -0.08 m, hallway lateral -0.12 m: walker about 0.1-0.2 m to the robot's right, i.e. near-centred, inside the estimate's noise. Ghost dropped at 0.43 m t+6.1. No stops, max lateral deviation 0.91 m. | Robot passed on my right side (after the localization check). |

- ~11:55 Brian's decision: the noghost version is good as it is (ghost dropped at 1.0 m, no "only behind the robot" variant). A
  person hidden behind the walker is to be handled by stop-and-beep, not by the slow-down: collision monitor stops the robot,
  `blocked_person_beep_node_front.py hold:=true` then beeps and holds (person within 1.0 m and +/-30° ahead in the last 1 s, or an
  unmapped LiDAR point within 1.0 m and +/-30° in the last 2 s, while the robot stands still with a goal active). Both nodes running.

## Why LEFT (walker's right) in both noghost runs (analysed ~12:00)

Side rule: walker's mean offset from the robot-goal line over 0.6 s after the lane starts; < -0.15 m (robot's right) -> robot keeps LEFT.
Offsets read: noghost run01 -0.32 m, run02 -0.21 m (critic log). Scratchpad `side.py`, `align.py` (session 9 Oct, not in the repo).

| Run | side | walker offset vs goal line, t+0.5..3.5 (fused / camera rays) | AMCL yaw error at start (scan-to-map fit) |
|---|---|---|---|
| rec15 run02 | RIGHT | +0.14 / +0.08 | -0.5° |
| rec15 run03 | RIGHT | -0.16 / -0.18 (critic: -0.06 in its window) | -0.5° |
| rec15 run04 | LEFT | -0.21 / -0.13 | +2.0° |
| noghost run01 | LEFT | -0.66 / -0.50 | **+3.5°** (mean scan-map distance 0.090 m -> 0.053 m when turned) |
| noghost run02 | LEFT | -0.24 / -0.10 | +2.0° |

A heading error e moves a walker at 6.4 m sideways by 6.4*sin(e): 2° = 0.22 m, 3.5° = 0.39 m, towards the robot's right for these
signs. Corrected, run04 and noghost run02 read about 0.0 m (centred -> RIGHT); noghost run01 about -0.27 m (still LEFT).
AMCL was re-initialised by hand at 11:46 (std 0.44 m / 14° right after). Yaw fit: 0.5° grid, LiDAR at t-1.5..0 s, mean distance
to the nearest occupied map cell capped at 0.5 m. Estimate, not ground truth.

## Pre-run localization check (from ~12:05)

New `workspace/scripts/check_localization.py`: ~2 s of scans placed on the map with the AMCL pose, grid search heading +/-8°
(0.25°) and position +/-0.20 m (0.05 m) for the best fit; OK if |heading| <= 1° and shift <= 0.10 m; `--fix` sends the corrected
pose on /turtlebot4/initialpose. Brian chose this over a larger side_switch_offset or always-keep-right.
- 12:05 robot at the start: AMCL heading 142.9°, fit says +3.75° (0.089 -> 0.048 m), "walker at 6.4 m placed 0.42 m too far to the
  robot's right" -> NOT OK. `--fix` -> heading 146.6°, recheck +0.25°, OK.
- 12:02 accidental start with run number 2 again (Ctrl+C before the goal): recorder stopped by the chat, folder renamed `VOID_headon_rec15noghost_run02_20261009_100013` (96 MB, VOID.txt inside). The real run 2 is `headon_rec15noghost_run02_20261009_095414`.

## Why the camera loses the walker (analysed ~12:20, scratchpad `camloss.py`)

Camera half field of view atan(125 px / 287.31 px) = 23.5°. Per run: last camera-confirmed fix, walker bearing, robot heading
(off the hallway), YOLO boxes (x1-x2 in the 250 px image) and log lines around it.

- **Out of view during the turn** (box slides to the image edge while the robot turns): rec15 run02 (box 131 -> 62 px, left
  edge, turn starting), rec15 run03 (95 -> 50 px), rec15 run04 (box 221-250 at t+2.76), noghost run01 (140 -> 205-248 px, robot
  +8..+25°, walker -16..-26°), noghost run02 (149 -> 185-224 px, robot +14..+37°, walker -19..-34°), noghost run03 (95 -> 0-49 px
  at t+4.0, robot -17°, walker 3.6 m at +16°).
- **In view but dropped by the tracker**: noghost run04 (t+3.86, box 104-148 px, mid-image) and run05 (t+3.19, box 105-154 px):
  YOLO detects the person but ByteTrack gives no track id ("REJECT track id:-1"), so the detection is not used. Both at the start
  of the fast turn (heading -14 -> -50° in 0.5 s), when the image moves sideways quickly.
- Before the turn, the camera-ray range lock ("refused x m (tracking 7.3 m)", then "release/motion-release 7.3 -> 5.6 m") holds
  an old range for up to ~0.9 s and then jumps (runs rec15 02, noghost 03, 05): this is what gives the KF its too-high speed.
- 12:19 second accidental start (old command, run number 4, CAMERA=1), Ctrl+C before the goal: recorder stopped by the chat, folder renamed `VOID_headon_rec15noghost_run04_20261009_101905`. The real run 4 is `headon_rec15noghost_run04_20261009_100916`.

## Would side_switch_offset 0.30 bring back the 5 Oct problem? (analysed ~12:25)

5 Oct `headon_lanegoalside_run01_20261005_100329` (threshold 0.30, robot kept RIGHT towards a walker standing 0.4-0.65 m to its
right): AMCL heading at the start +2.0° off (fit 0.071 -> 0.059 m), which places the walker ~0.22 m too far RIGHT, i.e. it does
NOT explain the under-reading. Walker offset over t+0.5..3.5: fused -0.59 m, camera rays -0.14 m; the critic's own 0.6 s window
read -0.26 m. So the 5 Oct walker really was on the right and the short window under-read it.
Critic-window readings today and then: centred walker -0.19 (noghost 06), -0.22 (noghost 05), 0.00 (03, 04); walker 0.4-0.65 m
right: -0.26 (5 Oct). They overlap; no single threshold separates them in the 0.6 s window. Over 3 s the fused offsets separate
better (-0.13 vs -0.59).

## Side threshold sweep (offline, ~12:45)

Scripts and output: `workspace/pytest/analysis/side_20261009/` (`sidesweep.py`, `runs.json`, `sidesweep_result.txt`; also
`align.py`, `side.py`, `camloss.py`, `wiggle.py`, `simhead.py`, `robhead.py` from today). The critic's side decision is recomputed
from each bag (KF estimate vs the robot-goal line anchored at the lane start, 20 Hz, side hold logic; the 6 cm commit ignored),
with raw positions and with positions corrected for the AMCL heading error found by `align.py`.
Check: raw inputs + the settings used at the time reproduce the actual side in all 11 runs (10 today at 0.6 s / 0.15 / hold 0.3,
5 Oct at 0.30 / no hold).

Key point: with the side hold, the FIRST sample sets the side; a longer window changes little (hold off + long window gave
erratic results). Loc-corrected offsets at the decision (first sample / 0.6 s mean): 5 Oct -0.26/-0.04, rec15 01 -0.33/-0.31
(lane at 2.8 m), 02 0.00/-0.01, 03 -0.16/-0.10, 04 +0.03/+0.04, noghost 01 -0.06/+0.08, 02 -0.13/-0.01, 03 +0.15/+0.01,
04 +0.01/0.00, 05 -0.20/-0.31, 06 -0.16/-0.14.
Correction to the ~12:25 note: corrected for its +2.0° AMCL error, the 5 Oct walker's first sample is -0.26 m (raw -0.49).

Wanted (Brian's rule): centred -> R; clearly right -> L; 5 Oct -> L (Brian had to yield when it kept right).
Loc-corrected decisions at window 0.6 s, hold 0.3:
- 0.15 (now): rec15 03 L and noghost 06 L although centred -> 2 unwanted.
- **0.20 and 0.25: all 11 as wanted** (5 Oct L, rec15 01 L, noghost 05 L, every centred run R).
- 0.30: 5 Oct R again (the 5 Oct problem comes back).
Margins are small: noghost 06 first sample -0.16 (vs 0.20), 5 Oct -0.26 (vs 0.25). 11 runs; not tested in simulation.

## rec20 in simulation (12:37-13:10)

10 head-on trials, sim rec15 config with side_switch_offset 0.20 (`..._hwsync_rec20_sim.yaml`, `run_rec20_test.sh`, critic 98844e8d):
**8 right / 2 left, same as rec15 (0.15)**; no side changes; closest 0.747 +/- 0.052 m (rec15 0.746 +/- 0.043); all goals.
Both LEFT trials were LEFT from the first sample (walker exactly centred, first estimate -0.22 m in trial09). In simulation the
first-sample error reaches the 0.20 threshold, so the threshold alone cannot remove the left passes. Details:
`~/thesis_social_navigation/notes/rec20_sim_20261009.md`.

## Afternoon: 0.20 vs 0.25 on the robot (plan, ~13:50)

- Simulation batch "walker 0.4 m right" stopped by the chat at 13:34 during trial 1 (Gazebo would load the laptop during robot runs);
  `VOID_headon_rec20_y-0.4_trial01`, rerun all 10 later (`run_rec20_y04_test.sh`).
- Create 3 base not communicating after charging (no odom/battery/dock topics, no undock action); Brian restarted it, resynced chrony,
  restarted turtlebot4.service: odom 22 Hz (delay 0.025 s), battery 95 %, scan 0.13 s; dock_status says is_docked false.
- Brian's plan: 3 runs at side_switch_offset 0.20 (`..._tf05_rec20_noghost_hw.yaml`, condition rec20noghost) and 3 at 0.25
  (`..._tf05_rec25_noghost_hw.yaml`, condition rec25noghost); pick the final value from the bags.
- 13:50 check: Nav2 was still on `..._rec15_noghost_hw.yaml` (live side_switch_offset 0.15) -> restart needed.
| 13:49 | `headon_rec20noghost_run01_20261009_114928` | **NOT COUNTED: perception had no map position.** Config rec20 noghost loaded (live side_switch_offset 0.2). YOLO detected the walker (202 messages on /person_positions_map) but camera_ray_person_node failed every lookup ("map and rplidar_link not part of the same tree"); /camera_ray_clusters, /person_positions_fused, /predicted_person_positions empty; /turtlebot4/odom not recorded. Odom -> base_link from the Create 3 was missing right after its restart. Critic saw no person; robot drove straight. ~13:55: TF tree complete again (map->rplidar_link, odom->base_link), perception restarted by Brian. Next counted 0.20 runs: rec20noghost 2-4. | It did not avoid me. |
- 13:54 accidental start of rec20noghost run 2 (Ctrl+C before the goal): recorder stopped, folder renamed `VOID_headon_rec20noghost_run02_20261009_115409`. Run 2 still to do.
| 14:04 | `headon_rec20noghost_run02_20261009_120453` | **Counted, 0.20.** Localization at the start: fit +0.0° (OK). Start 0.5 m behind the start point, heading 13° right of the hallway. Walker id 11 camera confirmed at 7.9 m. **Lane rule t+2.0 s, 6.97 m ahead, keeps RIGHT, fixed t+2.6 with offset +0.12 m** (3-s mean afterwards -0.20 / rays -0.14). Walker LiDAR-only from t+2, track lost after t+4. Swerve again hard: heading -94° at t+4.0, sideways to -0.83 m at t+7. No ghost drop, no occlusion slow-down, 0.16-0.31 m/s throughout, no stops. Nearest LiDAR ahead 0.55 m at t+7.6 (bearing -26°); nearest any direction 0.40 m t+8.5-9.0 behind the robot. Max lateral deviation 0.83 m, goal reached (29.2 s). | |
- ~14:15 beeping during the return to the start: the last return goal is a turn on the spot at the start; Brian sat at his
  table within 1 m in front, so blocked_person_beep_node_front.py (goal active + no headway + unmapped LiDAR point ahead) beeped
  and held the robot. Fix (new copy, original unchanged): `scripts/blocked_person_beep_node_front_atgoal.py` = never blocked while
  the robot is within 0.40 m of the newest goal on /turtlebot4/goal_pose (it only turns there). Also new, optional:
  `scripts/turn_in_place.py` (slow turn on the spot without a Nav2 goal, 3 warning beeps if something is within 1 m ahead).
| 14:13 | `headon_rec20noghost_run03_20261009_121336` | **Counted, 0.20.** Hold node `blocked_person_beep_node_front_atgoal.py` running. Localization at the start **-2.0° off** (fit 0.071 -> 0.043 m; check apparently not run or AMCL drifted on the return). Walker id 19 camera confirmed at 7.6 m. **Lane rule t+2.2 s, 5.95 m ahead, keeps RIGHT, fixed t+2.8 with offset +0.27 m** (3-s fused +0.43, rays +0.56; corrected for -2° about 0.22 m less, ~+0.05: centred, RIGHT correct either way). Swerve to -68°, sideways to -0.83 m; brief slow-down to 0.12 m/s t+6.0-6.5 while the walker went by; ghost dropped at 0.83 m t+6.1. Nearest LiDAR ahead 0.58 m (t+6.7), any direction 0.39 m behind-left (t+7.0). No stops, max lateral deviation 0.80 m, goal reached (26.2 s). | |
- 14:20 deleted at Brian's request (broken or not a run): `headon_rec20noghost_run01_20261009_114928` (no person data, analysis above kept), `VOID_headon_rec15noghost_run02_20261009_100013`, `VOID_headon_rec15noghost_run04_20261009_101905`, `VOID_headon_rec20noghost_run02_20261009_115409`, sim `VOID_headon_rec20_y-0.4_trial01`. Goal: 10 valid rec20noghost runs (2 so far: 02, 03).
- 14:25 **new start position (Brian): (-6.88, 13.41), heading 140°** (was (-6.80, 13.50), 141°; 0.12 m and 1° apart). Localization fixed there first (AMCL heading +2.5° off -> OK). Return: goal (-6.88, 13.41) heading -40° (qz -0.342 qw 0.940), then the same point heading 140° (qz 0.940 qw 0.342). Goal unchanged (-12.20, 17.75).
| 14:20 | `headon_rec20noghost_run04_20261009_122047` | **Counted, 0.20.** New start. Localization at the start 0.0°. Walker id 22 camera confirmed at 7.3 m. **Lane rule t+1.8 s, 6.49 m ahead, keeps RIGHT, fixed t+2.5 with offset -0.10 m** (first sample about -0.21 in the recomputation, 0.6 s mean -0.11). No stops, max lateral deviation 0.55 m (smallest swerve today), nearest LiDAR ahead 0.86 m, goal reached (25.3 s). | |
| 14:22 | `headon_rec20noghost_run05_20261009_122245` | **Counted, 0.20.** Localization 0.0°. Walker id 27 camera confirmed at 7.3 m. **Lane rule t+1.5 s, 6.61 m ahead, keeps LEFT, fixed t+2.2 with offset -0.23 m.** All measures agree the walker was ~0.2 m to the robot's right: first sample -0.28, 0.6 s mean -0.25, fused 3 s -0.21, camera rays -0.21, hallway lateral -0.21. LEFT is the rule's result for that offset (also at 0.25: first sample -0.28). Ghost dropped at 0.96 m t+6.1. No stops, max lateral deviation 0.92 m, nearest LiDAR ahead 0.87 m, goal reached (26.7 s). | |
| 14:25 | `headon_rec20noghost_run06_20261009_122539` | **Counted, 0.20.** Localization 0.0°. Lane t+2.0 s at 5.88 m, **keeps RIGHT, offset +0.31** (fused 3 s +0.27, rays +0.16). No stops, max lateral deviation 0.81 m, nearest LiDAR ahead 0.80 m, goal reached (25.6 s). | |
| 14:27 | `headon_rec20noghost_run07_20261009_122715` | **Counted, 0.20.** Localization 0.0°. Walker stood at ~7.4 m until ~t+2.5 (provisional lane at t-0.1, 7.54 m); lane again at t+4.6 at 5.87 m, **keeps RIGHT, fixed t+5.2 with offset +0.34** (late decision: walker started late). Before walking the walker stood ~0.4 m to the robot's right (fused t+0.5-3.5 -0.40). No stops, max lateral deviation 0.64 m, nearest LiDAR ahead 0.95 m, goal reached (26.6 s). | |
| 14:30 | `headon_rec20noghost_run08_20261009_122959` | **Counted, 0.20.** Localization 0.0°. A static held track (id 31, lidar_hold) 1.7 m behind-right of the start (Brian's table area), not in front, no effect. Walker id 32 camera confirmed at 7.3 m; lane t+2.1 s at 6.36 m, **keeps RIGHT, offset +0.05**. No stops, max lateral deviation 0.45 m (smallest today), nearest LiDAR ahead 0.87 m, goal reached (25.3 s). | |
| 14:32 | `headon_rec20noghost_run09_20261009_123149` | **Counted, 0.20.** Localization 0.0°. Lane late: t+2.8 s at 4.70 m (walker closed fast 6.8 -> 4.7 m in 1 s), **keeps RIGHT, offset -0.11**. Ghost dropped at 0.46 m t+6.0. No stops, max lateral deviation 0.76 m, goal reached (27.2 s). | |
- 14:40 Brian: in run 09 the robot passed on his RIGHT. Confirmed: critic "keeps RIGHT" (offset -0.11, fixed t+3.5 at ~4.7 m) and
  the global plan went right (-0.53 .. -0.82 m from t+3.0), but the controller turned LEFT (heading +25 -> +85° from t+3.5) and
  drove to lateral +0.87 m (left side), nearest LiDAR 0.33 m behind-left at t+7 (probably the left wall). Walker LiDAR-only from
  t+3.5, estimate drifting to the robot's LEFT (+0.26 -> +0.58 m) while the robot also moved left; ghost dropped at 0.46 m t+6.0.
  Only mismatch of the 8 rec20 runs (critic side vs robot's actual side: 02-08 all agree). Cause not established; candidates: late
  side decision (4.7 m, walker closing fast) with the LiDAR-only estimate drifting. Run counted as valid (a real outcome), flagged.
| 14:35 | `headon_rec20noghost_run10_20261009_123533` | **Counted, 0.20.** Localization 0.0°. Two tracks: id 37 (offset +1.18, well to the robot's left, not the walker) and the walker id 38; lane t+2.4 s at 5.36 m, **keeps RIGHT; walker's side fixed RIGHT with offset -0.13**. Occlusion slow-down t+6.2-7.6 (real walker going by), ghost dropped at 0.43 m t+6.6. Robot went RIGHT (lateral -0.52 m). No stops, nearest LiDAR ahead 0.78 m, goal reached (26.9 s). | |
| 14:38 | `headon_rec20noghost_run01_20261009_123757` (second run 1, after the broken one was deleted) | **NOT COUNTED, deleted: localization 2.0° off at the start** (fit 0.060 -> 0.045 m; check not run). Lane t+2.0 s at 6.36 m, **keeps LEFT** from the first sample: raw -0.22 (past -0.20), corrected +0.01 (would be RIGHT); 0.6 s mean raw -0.03 / corrected +0.18. Robot went LEFT to +1.21 m. Occlusion slow-down from t+5.5, ghost dropped at 0.34 m t+6.0. Wrong side caused by the heading error, as in noghost 01/02. | |
- 14:55 robot not moving: Nav2 commanded a turn (-1.14 rad/s), Create 3 wheel PWM at +/-32767 but motor current 0 mA and wheel speed 0; STALL hazard on right_wheel (and BACKUP_LIMIT) just before. Motor drive cut after a stall. Goal cancelled by the chat; Create 3 restart needed.
| 14:53 | `headon_rec20noghost_run01_20261009_125322` (third run 1) | **Counted, 0.20.** After the Create 3 restart (motor drive back, robot moving). Localization 0.0°. Walker id 0 camera confirmed at 7.6 m; lane t+1.9 s at 6.76 m, **keeps RIGHT, offset +0.35**; robot went RIGHT (lateral -0.38 m). No stops, no hazards, max lateral deviation 0.50 m, nearest LiDAR ahead 0.89 m, goal reached (26.2 s). **10th valid rec20noghost run.** | |

## Standing pair, wide hallway, rec20 noghost (from ~15:10)

Brian: hardware first, then simulation. Same Nav2 config (`..._tf05_rec20_noghost_hw.yaml`), same route (start (-6.88, 13.41)
140°, goal (-12.20, 17.75)). Pair as on 5 Oct: two people facing each other across the middle of the hallway, 0.8-1.0 m apart,
about 4.5 m from the start (5 Oct: A (-9.99, 15.16), B (-9.48, 15.85) and later about (-9.5, 17.1) / (-10.0, 16.5)).
Run script `scripts/run_scenario_hw_v3.sh` (copy of v3 with SCEN from the environment), condition pairwide / rec20noghost.
Stack at 15:10: Nav2 rec20 noghost, perception noghost, zone stack hwdet_mapprobe (group detector lidarhold + zone node simport),
hold node atgoal, retry node.
- 15:30 wide-pair start checked against the 5 Oct bags: the counted set pairwide_hwdetmap_run01-03 started at (-6.39, 13.64) 133°, (-6.51, 13.77) 143°, (-6.41, 13.67) 136° (the 14:50 "configured" start (-6.32, 12.64) was used only in the pilot pairwide_zone_run01). Robot sent to their mean **(-6.44, 13.69), heading 137°**, used as the wide-pair start today.
| 15:13 | `pairwide_rec20noghost_run01_20261009_131340` | **Wide pair, rec20 noghost, run 1.** Start at the 5 Oct counted start. **Group: `conversation` in all 85 messages, t-13.6 to t+28.2 s** (largest gap 1.1 s), buffer 0.40 (wide) in 83 of 85; members 0.89-1.07 m apart at s 4.2-4.5 m, l -1.57/-0.55 (right half of the hallway; 5 Oct counted set: s 4.35, l -1.22/-0.39). Zone mask non-empty in all 85. **Robot passed LEFT of the pair** (l +0.33 at the pair), max lateral deviation 0.45 m, 0.24-0.30 m/s, no stops, goal reached (24.0 s); nearest LiDAR return not on the map 0.82 m (nearer member, t+18.0). **A pair member got a lane:** track 8 at t+3.3 s (3.2 m ahead), keeps LEFT, offset -0.98, "robot already committed" (same side as the pass, so no effect); dropped as a ghost at t+14.9. Script `pytest/analysis/side_20261009/pair_run.py`. | |

## Standing pair, narrow section, rec20 noghost (from ~15:40)

5 Oct counted narrow set (pairnarrow_hwdetmap_run01/02/03): start (-1.16, 8.80) 143°, (-1.37, 8.88) 136°, (-1.00, 8.78) 138°;
goal (-6.80, 13.50) 141° (qz 0.943 qw 0.334); pair `conversation`, buffer 0.05 (narrow), members around (-4.0..-4.75, 10.8..12.0),
~1.2 m apart across the narrow section, ~4 m ahead. Robot driven to (-1.18, 8.82) 139° (90 s from the far end, one retry):
AMCL heading 5.75° off after the long drive -> fixed: (-1.34, 8.81) 141.4°, OK. Runs: SCEN=pairnarrow with GX/GY/GQZ/GQW for
the (-6.80, 13.50) goal.
| 15:27 | `pairnarrow_rec20noghost_run01_20261009_132713` | **Narrow pair, rec20 noghost, run 1.** Group `conversation` in all 131 messages (t-34.8 to t+29.6 s), buffer 0.05 (narrow) in 124, 0.40 in 7; members at s 4.6-4.9 m, l +0.68 / -0.65 (1.3 m apart) in a 2.2 m section (walls at l +0.92 / -1.30), i.e. across the whole section. **Robot passed BETWEEN the members** (l -0.02 at the pair), as in all three counted 5 Oct narrow runs (l -0.08, 0.00, +0.10; nearest return 0.44 / 0.51 / 0.36 m). Nearest return not on the map today **0.57 m**. **Both members got a lane:** track 26 (left member) at t+2.3 s, 3.36 m ahead, keeps RIGHT -> robot veered right to l -0.19 (t+5.5 s); track 27 (right member) at t+3.4 s, keeps LEFT -> robot back to the middle and straight through the gap (Brian: "at first not straight, then straight"). Max lateral deviation 0.27 m, 0.19-0.31 m/s, no stops, goal reached (28.6 s). | Not straight at first, then straight. |
- 15:50 Brian: wide scenario until 10 valid bags (1 so far: run01). Robot at the wide start (-6.42, 13.68) 135.2° after the localization fix (-1.75°), OK.
| 15:35 | `pairwide_rec20noghost_run02_20261009_133546` | **Counted, wide pair 2.** Localization 0.0°. Group `conversation` in all 89 messages (t-14.3 to t+29.1 s), buffer 0.40 in all 89; members 0.85-0.89 m apart at s 4.4 m, l -1.3 / -0.4. **Passed LEFT of the pair** (l +0.43), nearest return not on the map 0.81 m (nearer member, t+17.2). Lane for a pair member: track 28 at t-0.2 s, 4.66 m ahead, keeps LEFT (same side as the pass, no effect). Max lateral deviation 0.57 m, no stops, goal reached (25.3 s). | |
| 15:40 | `pairwide_rec20noghost_run03_20261009_134014` | **NOT COUNTED (Brian: ruined by a third person).** Pair fine: `conversation` in all 121 messages, buffer 0.40, members s 4.6 m, l -1.3 / -0.6; robot passed LEFT of the pair (l +0.46). But a third person walked at the robot at t+8.5-11 s around s 2.0 m: nearest LiDAR return not on the map **0.26 m** (t+9.5-10.0, ahead), robot turned right to -56° and kept 0.17-0.30 m/s; this person was never tracked as a person (only tracks 31/32 of the pair), so only the costmap/LiDAR obstacle layers reacted. Not usable for the pair set; close pass to an untracked person noted. Bag kept until Brian decides (delete per the standing rule or keep as an incident). | Ruined by a third person. |
| 15:44 | `pairwide_rec20noghost_run04_20261009_134403` | **Counted, wide pair 3 of 10.** Localization 0.0°. Group `conversation` in all 81 messages (t-10.5 to t+29.3), buffer 0.40 in all; members s 4.3 m, l -1.77 .. -0.51. **Passed LEFT of the pair** (l +0.32), nearest return not on the map 0.83 m (nearer member, t+17.8). **No lane for a pair member** (no lane lines in the critic log). Max lateral deviation 0.51 m, no stops, goal reached (24.6 s). | |
| 15:46-16:03 | `pairwide_rec20noghost_run05..10` (134638, 135114, 135453, 135715, 140019, 140349) | **All counted.** Localization 0.0° at every start. Group `conversation` in every message (75-166 per run), buffer 0.40 except 12 / 2 / 2 narrow messages in runs 05 / 08 / 10; run 08 had a 10.6 s gap in group messages at t-23 (before the goal; continuous during the run). Robot passed LEFT of the pair every time (l +0.41 / +0.41 / +0.55 / +0.43 / +0.46 / +0.54 at the pair), nearest return not on the map 0.87 / 0.87 / 0.86 / 0.85 / 0.84 / 0.77 m. Lane for a pair member in 05, 07, 08, 10 (keeps LEFT, "robot already committed", same side as the pass). No stops, goals reached (24.8-26.7 s). | |

Wide pair, rec20 noghost, valid runs 01, 02, 04-10 (9; run 03 not counted, third person): passed LEFT of the pair 9/9, never
between the members; nearest return not on the map 0.84 +/- 0.03 m (0.77-0.87); robot at the pair l +0.43 m on average; time to
goal 25.2 s; no stops; group `conversation` in every message of every run; lane for a pair member in 6 of 9 (01, 02, 05, 07,
08, 10), always on the side already taken. One more run needed for 10.
- 16:10 deleted at Brian's request: `pairwide_rec20noghost_run03_20261009_134014` (third person; analysis above kept). Run 3 to be recorded again.
| 16:08 | `pairwide_rec20noghost_run03_20261009_140800` (rerun) | **Counted, 10th valid wide run.** Localization -0.5° (OK). Group `conversation` in all 83 messages (t-10.7 to t+30.0), buffer 0.40 in 81; members s 4.0 m, l -1.80 .. -0.24. **Passed LEFT of the pair** (l +0.50), nearest return not on the map 0.79 m. Lane for a pair member (track 74, keeps LEFT, already committed). Max lateral deviation 0.70 m, no stops, goal reached (26.2 s). | |

**Wide pair, rec20 noghost, 10 valid runs (01-10):** passed LEFT of the pair 10/10, never between the members; nearest return
not on the map 0.83 +/- 0.03 m (0.77-0.87); robot at the pair l +0.44 m on average; time to goal 25.3 s; no stops; localization
within 1° at every start; group `conversation` in every message of every run; lane for a pair member in 7 of 10 (01, 02, 03,
05, 07, 08, 10), always on the side already taken.
- 16:15 narrow pair set, to 10 valid runs (1 so far: run01). Robot at the narrow start (-1.20, 8.76) 142.4° after the localization fix (+2.75°), OK.
| 14:16-14:26 | `pairnarrow_rec20noghost_run02..05` (141632, 142131, 142417, 142637) | **Counted, narrow 2-5.** Localization 0.0 / +0.5 / +0.5 / 0.0°. Group detected in every run, buffer 0.05 (narrow) in 151/155, 129, 126, 123 messages; type `conversation` throughout except run 02 (64 `conversation`, 91 `side_by_side`; flagged). Pair at s 3.7-4.8 m across the section; robot passed BETWEEN the members every time (l -0.07 / +0.14 / -0.10 / +0.14). Nearest return not on the map near the pair (s pair-1.5..pair+1.0): 0.54 / 0.40 / 0.61 / 0.44 m (run 01: 0.57). (Without that window runs 02-04 pick up something unmapped near the goal end, s 6.1-6.4 m, 0.29-0.43 m.) Lanes for pair members in 02, 03, 04 (one in 03 with a short occlusion slow-down t+8.2-9.7). No stops, max lateral deviation 0.18-0.34 m, goals reached (25.9-29.8 s). | |
| 14:29 | `pairnarrow_rec20noghost_run06_20261009_142953` | **Failed, deleted:** no goal reached Nav2 (no plan, no cmd_vel, no Nav2 log lines); the script stopped before the goal, most likely at the camera check. Pair detected (`conversation`, 54 messages). | Failed. |
| 14:31 | `pairnarrow_rec20noghost_run07_20261009_143134` | **Failed (Brian), deleted.** Wi-Fi burst at the start (collision monitor ignoring 1.0-1.15 s old scans). Group 117 `conversation` / 46 `side_by_side`. The robot stopped about 1 m before the pair (s 3.4-3.7 m, pair at s 4.7) from t+13.6 s: stop-and-beep hold ON t+17.1 (unmapped LiDAR within 1 m ahead), off t+22.3, on again t+23.3; then passed between the members (l +0.08), nearest return near the pair 0.62 m; 3 stops, 45.2 s of motion. | Failed. |
- 16:45 deleted at Brian's request: narrow runs 06 and 07 (analysis above). Narrow valid: 01-05.
| 14:36 | `pairnarrow_rec20noghost_run07_20261009_143622` (second run 7) | **Not counted, deleted (Brian).** Localization 0.0°, group `conversation` in all 93 messages (narrow buffer in 81). Robot crept between the pair t+14-39 s at 0.03-0.10 m/s ("Failed to make progress" t+26.5), then the stop-and-beep hold (unmapped LiDAR ahead) held it 12.6 s just past the pair (s 5.06) and again from t+53 s; Nav2 goal aborted t+60.2, resent by the retry node t+63.2, reached t+81.8 (81.5 s of motion, 5 stops). Cause: an object not on the map at about (-6.1, 12.9), mid-corridor ~1 m before the narrow goal; already passed at 0.29-0.43 m near the goal end in runs 02-04, absent in run 01 (13:27). | Delete. |
| 14:40 | `pairnarrow_rec20noghost_run06_20261009_144039` | **Counted, narrow 6.** Localization 0.0°. Group `conversation` in all 131 messages, buffer 0.05 in all; pair at s 4.7 m, l -1.25 / +0.67. Passed BETWEEN the members (l -0.12), nearest return not on the map 0.52 m (at the pair; nothing near the goal any more: the object at (-6.1, 12.9) is gone). No lane for a pair member, no hold events, no stops, max lateral deviation 0.18 m, goal reached (28.0 s). | |
| 14:47 | `pairnarrow_rec20noghost_run07_20261009_144703` (third run 7) | **Failed, deleted (Brian):** no goal reached Nav2 (no plan, no cmd_vel), like the first run 06; the script stopped before the goal (camera check most likely). | Delete. |
| 14:48 | `pairnarrow_rec20noghost_run08_20261009_144819` | **Counted, narrow 7 of 10.** Localization -0.5°. Group `conversation` in all 91 messages, buffer 0.05 in 86. Passed BETWEEN the members (l -0.12), nearest return not on the map 0.53 m. Lane for a pair member (track 1, keeps RIGHT, already committed). No holds, no stops, max lateral deviation 0.29 m, goal reached (28.0 s). Track ids restart at 1 (perception restarted before this run). | |
| 14:52-15:01 | `pairnarrow_rec20noghost_run07_20261009_145221`, `_run09_20261009_145731`, `_run10_20261009_150100` | **All counted.** Localization +0.5 / 0.0 / 0.0°. Group `conversation` in all 145 / 135 / 145 messages, narrow buffer in all but 2 / 2 / 1. Passed BETWEEN the members (l +0.08 / -0.04 / +0.02), nearest return not on the map near the pair 0.49 / 0.55 / 0.54 m. No hold events. Run 07: 1 stop >= 0.5 s, 34.6 s of motion (reached the pair at t+25 s, later than the others). Run 09: lanes for BOTH members (track 19 RIGHT, track 20 LEFT, as in run 01), 32.9 s. Run 10: lane for one member (LEFT), 28.7 s. | |

**Narrow pair, rec20 noghost, 10 valid runs (01-10):** passed BETWEEN the members 10/10 (section 2.2 m wide, the pair across it;
same as all three counted 5 Oct runs); nearest return not on the map near the pair 0.52 +/- 0.06 m (0.40-0.61; 5 Oct 0.36-0.51);
time to goal 29.2 s (25.9-34.6); stops: one run (07, one stop); no stop-and-beep holds; localization within 1° at every start;
group detected in every message of every run, `conversation` except run 02 (91 of 155 `side_by_side`); narrow buffer throughout;
lanes for pair members in 7 of 10 (both members in 01 and 09).
