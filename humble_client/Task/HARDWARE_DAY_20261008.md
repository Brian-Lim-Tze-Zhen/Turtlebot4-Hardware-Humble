# Hardware day 8 Oct 2026: run log

Started at the second head-on noghost run; the morning runs (`headon_v4front_run01`–`03`,
`headon_v4noghost_run01`) are in `workspace/bags/` but were not logged here.

## Set-up (head-on, v4noghost)

- Nav2: `launch/lab2_bringup_cm.launch.py` with `config/social_nav2_headon_F_hwreq_block_approach_hw.yaml` (`vx_min` 0.0)
- Perception: `launch/perception_camray_lane_noghost_bringup.launch.py`
- Zone stack: `launch/costmap_filter_info_hwdet_mapprobe.launch.py`
- Beep: `scripts/beep_retry_node.py`, `scripts/blocked_person_beep_node_front.py -p hold:=true`
- Run command (laptop): `scripts/run_headon_hw.sh v4noghost <run> 5`, recorder `record_hw_run_v4.sh` (camera not recorded)
- Start: near end (-6.80, 13.50), heading 141°; goal: far end (-12.20, 17.75), same heading
- Return to start in two goals (facing the near end, then turn on the spot), because without reversing a single goal gets stuck turning in the last 0.35 m

## Runs

| Time | Bag | Result | Brian's note |
|---|---|---|---|
| 13:56 | `headon_v4noghost_run02_20261008_115641` | Goal reached in 30.5 s, 6.97 m; no reversing or hazards. One identity, seen first at 7.37 m, last published at 2.06 m. Sideways deviation up to 0.82 m. Nearest unmapped LiDAR point 0.69 m behind-right at t+9.2 s (probably the walker after the pass). **Stop t+1.1 to t+3.3 s: TF timing fault**, not a reaction to the walker: the controller aborted the path eight times ("Lookup would require extrapolation into the future"). Start tone sent (in the bag) but not heard from the far end. | Walked straight. The robot moved, stopped for about 1 s and continued, so the walker stopped for a moment at the start and then walked straight on. |
| 14:04 | `headon_v4noghost_run03_20261008_120448` | **NOT VALID** (Brian: someone else walked in). Goal reached in 31.1 s, 7.19 m, no reversing or hazards, one 0.7 s stop at t+21.5 s; 3 identities (0, 1, 2), a person 0.52 m from the robot 8 s before the start. No TF timing fault after the resync. Bag kept until Brian decides. | Ruined: another person walked in. To be repeated as run 3. |
| 14:08 | `headon_v4noghost_run03_20261008_120759` | Repeat of run 3. Camera silent at the pre-goal check, restarted by the run command (22 Hz); goal about 43 s after the bag started. Goal reached in 27.8 s, 7.00 m; no stops, reversing or hazards; no TF timing fault. Walker seen first at 5.55 m (t+2.9 s), last published at 2.00 m (t+5.8 s), robot at 0.27 m/s. Sideways deviation up to 0.74 m. Nearest unmapped LiDAR point 0.57 m at t+7.5 s, bearing +136 deg (walker passing on the robot's left, behind). | Walked straight; the robot passed on Brian's left. |
| 14:17 | `headon_v4noghosttf05_run01_20261008_121720` | **Config `..._approach_tf05_hw.yaml`** (controller `transform_tolerance` 0.5, Nav2 restarted 14:15). Goal reached in 28.0 s, 6.98 m; no stops >= 0.5 s, no reversing or hazards. One identity throughout, standing at 7.6 m before the start, last published at 3.8 m (t+5 s). Sideways deviation up to 0.89 m. Nearest unmapped LiDAR point 0.59 m at t+7.1 s, bearing +112 deg (walker on the robot's left). Controller still aborted twice at t+21.8 s (one "extrapolation into the future" only 8 ms ahead, so the 0.5 s tolerance did not prevent it), restarted at once; collision monitor TF failures 24, scan ignored 10. | Walked straight; the robot passed on Brian's left. Start signal (warble) not heard. |
| 14:21 | `headon_v4noghosttf05_run02_20261008_122118` | tf05 config, no start sound. Start heading 154 deg (13 deg left of the hallway). Camera silent at the pre-goal check, restarted by the run command; goal about 40 s after the bag started. Goal reached in 29.7 s, 7.01 m; no stops >= 0.5 s, no reversing or hazards; **no controller aborts**. One identity throughout (held as lidar_hold while standing). Robot vx 0.17 m/s at the closest tracked point (2.96 m, t+5.8 s). Sideways deviation up to 0.83 m. Nearest unmapped LiDAR point 0.61 m at t+8.1 s, bearing +116 deg (walker on the robot's left). Collision monitor: scan ignored 47 times, TF failures 20. Battery 78 %. | Walked straight; the robot passed on Brian's left. |
| 14:27 | `headon_v4noghosttf05_run03_20261008_122708` | tf05 config, no start sound. Camera silent at the pre-goal check, restarted by the run command; goal about 38 s after the bag started. Goal reached in 27.3 s, 6.74 m; no stops, reversing or hazards; **no controller aborts**; collision monitor scan ignored once, TF failures 18. Walker held as id 5 while standing, new identity 6 at t+1.7 s. Lane rule at t+3.0 s with the walker 5.88 m ahead: **robot keeps LEFT** (walker's mean offset -0.58 m, i.e. to the robot's right). Track last published at 4.53 m (t+4.8 s). Sideways deviation only 0.45 m. Nearest unmapped LiDAR point 0.67 m at t+7.7 s, bearing -90 deg (walker on the robot's right). Battery 77 %. | Walked straight; the robot passed on Brian's right. |

## Notes

- 14:00 clock check after run 2: robot stamps 0.4 to 0.8 s behind the laptop (scan median 0.57 s, odom 0.40 s); laptop chrony 2 ms off NTP. On 5 Oct after the robot resync the scan delay was 0.13 to 0.16 s.
- 14:03 after Brian's robot resync: scan delay median 0.16 s (0.14 to 0.28 s); odom took about 20 s to come back after the turtlebot4 service restart.
- Start signal changed after run 2 to a two-pitch warble of about 2 s (880/1320 Hz, no silent notes).
- 14:15 return-trip log check (Nav2 ~/.ros/log): every drive since 14:02 had collision-monitor TF failures and scans ignored as >1 s old; controller aborts at 14:05:33 (x2) and 14:10:36; BlockedHold 14:10:50-53 during the turn at the start (Brian about 2 m away). Test config `config/social_nav2_headon_F_hwreq_block_approach_tf05_hw.yaml` created (controller transform_tolerance 0.2 -> 0.5).
- 14:21 start signal: Brian hears both the warble and a single tone at the robot, but does not want it that loud; run_headon_hw.sh plays no sound by default from now on (BEEP=1 to switch it back on). The walker starts when the robot starts moving.
