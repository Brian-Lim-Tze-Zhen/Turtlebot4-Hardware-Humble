# Prompt to continue the hardware day (paste into a new chat opened in ~/Desktop/Turtlebot4)

I am Brian, a Master's student (DIT). My thesis "Lightweight Socially-Aware Navigation via Human Motion Prediction and Scene Understanding" is due on 15 October 2026. Today, 5 October 2026, I have the physical TurtleBot 4 for the day. An earlier chat (opened in the simulation repository) got the robot running and recorded the empty drive. Continue from there. I am at the robot, you are on the laptop. Work step by step, one command at a time, and wait for my answer.

## Read these first

1. `~/Draft - Thesis/HANDOFF_hardware_day.md` - the full plan for the day (rules, Step 2 test runs, Step 3 photo list, Step 4 thesis update and the email to the examiner). It still applies in full; this file only adds the current state.
2. `~/Desktop/Turtlebot4/humble_client/Task/HARDWARE_DAY_20261005.md` - today's run log. Keep it up to date after every run (time, bag, what happened, anything unusual), plus the photo table.
3. The memory files of the simulation project, which this new chat does not load by itself: `/home/brian/.claude/projects/-home-brian-thesis-social-navigation/memory/` (`project_thesis_draft_v2.md`, `feedback_thesis_results_provable.md`, `feedback_keep_thesis_scripts.md`, `project_headon_avoid_sim.md`, `project_static_streak_hw_test.md`).

## Rules (short form; details in the first file)

- Back up before editing any existing hardware script or config: `<name>.bak_<YYYYMMDD>_<reason>`. Never delete a script or config that produced thesis results. Show me a change before making it.
- Every number in the thesis must be provable from a bag plus a script plus a saved output.
- Do not commit or push unless I ask.
- Safety first: keep the speed limits of the config; I stay near the robot.

## Decisions I made in the earlier chat

- Today runs the CURRENT hardware pipeline first. The port of the simulation head-on lane rules (extended SocialCritic, `hwreq` configs, body-disk cloud) comes only after the five fixed-configuration head-on runs, on a new branch from the tag below, and only if time allows. The hardware SocialCritic (493 lines) has none of the lane rules; the simulation one has 1229 lines.
- `debug_crops_labels/` and `debug_crops_vlm_report/` stay uncommitted (they contain camera crops of people).

## State of the hardware repository

- Repository root `~/Desktop/Turtlebot4`, remote `Turtlebot4-hardware` on GitHub. Branch `headon-tuning-2oct` at `5df1620`, pushed. Tag `hw-before-lane-port-20261004` marks this state, pushed.
- Uncommitted new files from today: `humble_client/workspace/scripts/record_hw_run.sh`, `humble_client/workspace/scripts/quick_bag_check.py`, `humble_client/Task/HARDWARE_DAY_20261005.md`, this file.
- Only backup made today: `scripts/record_social_bag.sh.bak_20261004_hwday_topics` (the original was not edited).
- `Task/NEXT_SESSION.md` is from 8 September and describes the old LiDAR-detector pipeline; it is to be rewritten at the end of the day (Step 4).

## How the robot is run

- Container `thesis_social_nav_real` (`docker compose up -d` in `humble_client`; ROS 2 Humble, CycloneDDS, domain 1). `run_humble_client.sh` is NOT the robot container. The workspace is mounted at `/root/thesis_social_navigation_ws`. You can run read-only checks yourself with `docker exec thesis_social_nav_real bash -c 'source /opt/ros/humble/setup.bash; ...'`; use `-e ROS_DOMAIN_ID=77` for offline bag analysis.
- All robot topics are under `/turtlebot4/`. The zone mask topic is `/social_zone_map`. Container time is UTC, 2 h behind local time; bag folder timestamps are UTC.
- The camera publishes nothing while the robot is docked.
- Running right now (leave them running): Nav2 + localisation + RViz (`launch/lab2_bringup.launch.py`, map `map/hall_map_name.yaml`, params `config/social_nav2_ablation_F_socialzone_keepout.yaml`, log `logs/lab2_bringup_20261005_075009.log`) and perception (`launch/perception_camray_bringup.launch.py`, log `logs/perception_20261005_080415.log`).
- The Nav2 log repeats "KeepoutFilter: Filter mask was not received" because the zone node is not started for head-on runs. Planning works regardless (same as 2 October).
- The installed SocialCritic was built on 1 October after its last source edit; no rebuild needed for the current pipeline.

## Fixed configuration for the head-on block (do not change between counted runs)

- Nav2 config `config/social_nav2_ablation_F_socialzone_keepout.yaml`, camera-ray perception, map `hall_map_name`.
- Start (far end), robot facing down the hallway: x -12.20, y 17.75, qz -0.334, qw 0.943.
- Goal (near end): x -6.80, y 13.50, qz -0.334, qw 0.943. Hallway length about 7.1 m.
- Walker starts at the near end in the middle of the hallway and walks towards the robot.

Commands (inside the container, in `/root/thesis_social_navigation_ws` with `install/setup.bash` sourced):

```bash
# turn the robot around / return it to the start
ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -12.20, y: 17.75, z: 0.0}, orientation: {z: -0.334, w: 0.943}}}"

# record one run: <scenario> <condition> <run_no> <config>; wait for "recording ... - Ctrl-C to stop" BEFORE sending the goal
./scripts/record_hw_run.sh headon pilot 1 config/social_nav2_ablation_F_socialzone_keepout.yaml

# goal for the run
ros2 topic pub --once /turtlebot4/goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -6.80, y: 13.50, z: 0.0}, orientation: {z: -0.334, w: 0.943}}}"
```

`record_hw_run.sh` names the bag `<scenario>_<condition>_runNN_<timestamp>`, records everything except image streams and `/trajectories` (27 MB/s) but keeps the compressed camera preview and `camera_info`, dumps the live parameters of both costmaps, controller, planner, navigator, smoother, AMCL and the perception nodes into `params/`, copies the Nav2 config and node scripts, and on Ctrl-C runs `quick_bag_check.py`, saved as `quick_check.txt` in the bag folder (topics and counts, motion, reversing time, stops, bumper events, nearest LiDAR return ahead, person identities). Read that file after every run. About 3.5 MB/s; 21 GB were free this morning.

## Done so far today

- Bring-up passed: robot reachable, clocks about 0.2 s apart, LiDAR 7.3-7.7 Hz, camera 29-30 Hz, TF chain complete, Nav2 active, both costmaps contain the person-cloud layer, every perception node publishes (standing-person test at 3.6 m). Battery was 58 % at 09:50 - watch it and dock during breaks.
- `empty_cloudon_run01_20261005_080504`: empty hallway, 6.68 m in 23.8 s, no reversing, no stops, no hazards, person cloud empty, recording complete. (An earlier partial bag of the same name was deleted at my request.)
- The robot was last at the far end (-12.02, 17.57), facing away from the hallway. The turn-around command above had been given to me but I have not confirmed running it.

## Next step: the pilot, then the plan from the first file

1. **Pilot, 1-2 runs** (`headon pilot 1`), slow walker, ready to step aside. Reason: the last 2 October run with these settings (`headon_F_wz07_infl035_kfq_20261002_145103`) hit the walker (bumper right at 8.3 s, 1.4 s reversing) because the person was range-locked at about 7 m. The fix, the motion release in `camera_ray_person_node.py` (commit `7d94403`), has only been checked in replay and never driven. If a pilot shows contact or a late detection, diagnose from the logs and the bag before changing anything.
2. **Five counted head-on runs**, `headon cloudon 1` to `5`, identical settings, normal walking pace.
3. **Person cloud on versus off**, same day: condition `cloudoff`, using a config without the person-cloud layer (check `config/social_nav2_headon_A_ablation.yaml` and `social_nav2_baseline_no_people.yaml` against the F config first and tell me which differs only in the cloud layer). The parameter dump in each bag folder is the proof of the condition.
4. Then, in the order of the first file: standing pair wide and narrow (needs the zone stack: `launch/social_perception_bringup.launch.py` or the camera-ray equivalent, and check live whether the pair is published as `conversation` or `side_by_side`; on 1 October it was `side_by_side`), stationary hold (it is on in the identity node: gate 0.35 m, miss 1.0 s, max 30 s; look for source `lidar_hold` in `/person_positions_fused`), the forward ellipse decision (still active in `predicted_person_cloud_node_lidar.py`: a = 0.60 + 0.75 x speed, b = 0.40), blocked corridor with beep.
5. Lane-rule port only after that, if time allows.

## Photos (remind me during the day)

None taken yet. The full list is in the first file. The hallway is set up now, so ask me early for: the empty hallway from the start position towards the goal, the robot three-quarter front view, side view, sensor close-up, laptop with RViz next to the robot, floor marks. During runs: a fixed-camera video of one head-on run, RViz screenshots with the person cloud and the bending plan, camera image with detection box. Save to `~/Draft - Thesis/figures_v2/screenshots/` and note time, bag and consent in the run log.

## End of the day

Step 4 of the first file: analyse the bags (`~/Draft - Thesis/hardware_analysis/hw_extra.py`, `workspace/pytest/analysis/headon_metrics.py`), update `thesis_v2.md` (back it up first), rebuild the Overleaf zip and preview PDF, draft the email to Prof. Dr. Dmitrii Dobriborsci, rewrite `Task/NEXT_SESSION.md`.
