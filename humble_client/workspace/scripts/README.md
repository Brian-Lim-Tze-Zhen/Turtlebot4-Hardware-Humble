# scripts

Nodes, run scripts and checks of the robot. The folder also holds the earlier
versions, because the bags of the earlier robot days refer to them; a newer
version is a copy with a suffix (`_v2`, `_noghost`, `_front_atgoal`, ...),
and the header of each file says what it changed.

Nodes are started with `python3 <file>`, not `ros2 run`, and need the
`/tf` and `/tf_static` remaps to `/turtlebot4/...` (see `HOW_TO_RUN.md`,
section 5).

## Final configuration (12 October 2026)

Started by `launch/robot_day_all.launch.py`:

| File | Role |
|---|---|
| `yolo_leg_detector_lidar.py` | YOLO pose and ByteTrack on the OAK-D image |
| `camera_ray_person_node.py` | turns each detection into a ray from the camera and measures its distance with the LiDAR |
| `camera_ray_identity_node.py` | keeps track identities across camera and LiDAR |
| `human_kf_predictor_lidar_noghost.py` | Kalman-filter prediction; drops a coasting track that runs into the robot |
| `predicted_person_cloud_node_lidar_lane.py` | the predicted people as a point cloud for the costmaps |
| `person_marker_publisher.py` | markers for RViz |
| `social_group_detector_node_lidarhold.py` | finds a standing pair, publishes `/social_groups` (imports `open_clip`, but its MobileCLIP fallback never runs) |
| `social_zone_costmap_node_simport.py` | the simulation's zone node: the graded social zone around a group |
| `blocked_person_beep_node_front_atgoal.py` | holds the robot and beeps while a person blocks it (`hold:=true`) |
| `beep_retry_node.py` | sends the goal again after Nav2 gives up while blocked |

Run tools, used on the laptop or in the container:

| File | Where | Role |
|---|---|---|
| `run_scenario_hw_v4.sh` | laptop | one recorded run (`SCEN=headon`, `pairwide`, `pairnarrow`); calls the two below |
| `record_hw_run_v4.sh` | container | the bag recorder of a run (started by `run_scenario_hw_v4.sh`) |
| `check_localization.py` | container | checks the AMCL pose against the map, `--fix` corrects it once |
| `check_group.py` | container | waits for a `conversation` group before a pair run and confirms it with two beeps |
| `nav_goal_wait.py` | container | sends the goal on `/turtlebot4/goal_pose` and waits for its result |
| `quick_bag_check.py` | container | checks that a bag has the required topics |

## Earlier versions

| Files | Replaced by, or what it was |
|---|---|
| `run_headon_hw.sh`, `run_headon_hw_v2.sh`, `run_headon_hw_v3.sh`, `run_scenario_hw_v3.sh` | `run_scenario_hw_v4.sh` (each header lists what the next version added) |
| `record_hw_run.sh`, `record_social_bag.sh` | `record_hw_run_v4.sh` |
| `human_kf_predictor_lidar.py` | `human_kf_predictor_lidar_noghost.py` |
| `predicted_person_cloud_node_lidar.py` | `predicted_person_cloud_node_lidar_lane.py` |
| `blocked_person_beep_node.py`, `blocked_person_beep_node_front.py` | `blocked_person_beep_node_front_atgoal.py` |
| `social_group_detector_node.py`, `social_group_detector_node_lidarhold_simport.py` | `social_group_detector_node_lidarhold.py` |
| `social_zone_costmap_node.py` | `social_zone_costmap_node_simport.py` |
| `lidar_person_detector.py`, `identity_fusion_node_lidar.py` | the camera-led pair `camera_ray_person_node.py`, `camera_ray_identity_node.py` |
| `social_group_cloud_node.py` | robot version of an early group cloud; replaced by the zone node and the KeepoutFilter |
| `social_zone_speed_limiter.py` | capped the speed inside a conversation zone; not used by the final configuration |
| `perception_camray_bringup.launch.py` | `launch/perception_camray_lane_noghost_bringup.launch.py` |
| `test_facing_vlm.py` | the rejected VLM facing test (`debug_crops_vlm_report/`) |
| `pair_pass_analysis.py` | one-off analysis of the robot oscillating in front of a pair; the analyses of the thesis are in `pytest/analysis/` |
