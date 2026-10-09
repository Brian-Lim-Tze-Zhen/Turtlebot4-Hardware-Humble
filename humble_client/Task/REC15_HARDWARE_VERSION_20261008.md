# Recommended configuration (rec15) on the robot — 8/9 Oct 2026

Same configuration as the simulation `ros2_ws/config/social_nav2_headon_F_hwreq_block_blockedhold_approach_hwsync_rec15_sim.yaml`
(thesis_social_navigation repo). Nothing original was edited; everything is new files.

## New files
| File | What |
|---|---|
| `workspace/src/social_critic_lane_hold/` | Copy of `social_critic_lane` (unchanged, lib md5 25712f78) with `side_hold_offset`. Class `mppi::critics::SocialLaneHoldCritic`, library `libsocial_critic_lane_hold.so` (md5 14ee9bdd, built 9 Oct in thesis-social-nav-humble-real). Log lines keep the `SocialLaneCritic:` prefix. |
| `workspace/config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_hw.yaml` | Copy of `..._approach_tf05_hw.yaml` (unchanged, md5 35766a28): critic `SocialLaneHoldCritic`, side_hold_offset 0.3, side_switch_offset 0.15, lane_block_overlap 0.45, lane_block_width 0.8, pass_side_margin 0.85, pass_side_max_offset 0.95. Kept from tf05: side_ref_goal_topic goal_pose, controller transform_tolerance 0.5, GoalAngleCritic 0.25, collision monitor + BlockedHold. |
| `workspace/bags/replay_rec15_port/` | Offline check: 5 robot bags replayed through the Humble critic (out/humble_rec15/) — same decisions as the Jazzy simulation critic with the rec15 sim config on the same bags (tracks, sides, timing; lane start within 0.01 m). |

## Jazzy -> Humble differences (handled)
- Humble MPPI gives the critic no goal: goal line from `goal_pose` (topic, published by `scripts/nav_goal_wait.py` / RViz "Nav2 Goal"); beyond-goal rule uses the end of the path. Simulation uses the controller goal. Offsets differ by a few cm; sides were identical in the check.
- Collision monitor: fixed BlockedHold square switched by `blocked_person_beep_node_front.py`, `max_points` (Humble) instead of `min_points` — as in tf05.
- Odometry `odom_reliable` (robot relay).

## Build (once)
    docker exec -it thesis_social_nav_real bash
    cd /root/thesis_social_navigation_ws && colcon build --packages-select social_critic_lane_hold && source install/setup.bash

## Run (head-on, as on 8 Oct)
    ros2 launch /root/thesis_social_navigation_ws/launch/lab2_bringup_cm.launch.py \
      params_file:=/root/thesis_social_navigation_ws/config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_hw.yaml
    ros2 launch /root/thesis_social_navigation_ws/launch/perception_camray_lane_noghost_bringup.launch.py
    python3 /root/thesis_social_navigation_ws/scripts/blocked_person_beep_node_front.py --ros-args -p hold:=true \
      -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
    python3 /root/thesis_social_navigation_ws/scripts/beep_retry_node.py --ros-args \
      -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static
    # laptop:
    CFG=config/social_nav2_headon_F_hwreq_block_approach_tf05_rec15_hw.yaml workspace/scripts/run_headon_hw.sh rec15 <run> 5

Check in the Nav2 log at start: "side hold: on ... [SocialLaneHoldCritic]" and "pass side measured against the robot-goal line (goal from goal_pose)".
Resync the robot clock (chrony) before the session.

## Simulation result of this configuration (8 Oct)
Head-on 10 runs 0.746 +/- 0.043 m (8 right / 2 left, no side flip, plan on the pass side 10/10, wall clearance >= 0.140 m);
narrow pair 6 runs 0.700 +/- 0.020 m, wide pair 6 runs 0.908 +/- 0.055 m, no lane for a pair member. v4: 0.740 / 0.689 / 0.870.
Known: a single noisy first sample can decide the side of a near-centred walker (2 of 10 went left).
Open (perception): on robot bags of 5 Oct a standing pair member still got a lane in 3 of 8 runs (estimate jumps).
