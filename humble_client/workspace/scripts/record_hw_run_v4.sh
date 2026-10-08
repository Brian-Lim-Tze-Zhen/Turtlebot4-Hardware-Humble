#!/usr/bin/env bash
# record_hw_run_v4.sh - 8 Oct 2026 copy of record_hw_run.sh (kept unchanged) that
# records ONLY the topics the analysis uses instead of everything (-a).
# The -a bag of headon_v4front_run01 was 301 MB for 88 s; 77 % of it were
# costmap voxel grids, costmap_raw duplicates and the follow_path /
# compute_path_to_pose action status, none of which any analysis reads.
# Recording a list also stops the bag from pulling every robot topic
# (IR, cliff, IMU, mouse, ...) over the Wi-Fi.
# Also added: the blocked-beep nodes are copied and the collision monitor's
# parameters are dumped. The camera preview is NOT recorded by default (8 Oct); CAMERA=1 adds it (+28 MB/run).
#
# Original header:
# record_hw_run.sh - record one hardware run with everything the offline
# analysis needs. Successor of record_social_bag.sh (kept unchanged).
#
# What it adds over record_social_bag.sh:
#   - bag name says scenario, condition and run number
#   - the compressed camera preview and camera_info are recorded
#   - parameters of BOTH costmaps (and planner, BT navigator, smoother,
#     perception nodes) are dumped, so a condition such as "person cloud
#     off" can be proven from the bag folder
#   - the Nav2 config file that was launched is copied next to the bag
#   - a quick check of the bag runs when the recording is stopped
#
# USAGE (inside the container, one run per call, Ctrl-C to stop):
#   ./record_hw_run_v4.sh <scenario> <condition> <run_no> <nav2_config.yaml>
#   CAMERA=1 ./record_hw_run_v4.sh headon v4front 2 config/social_nav2_headon_F_hwreq_block_approach_hw.yaml
#
# Everything lands in bags/<scenario>_<condition>_runNN_<timestamp>/.

set -uo pipefail

if [ $# -lt 4 ]; then
  echo "usage: $0 <scenario> <condition> <run_no> <nav2_config.yaml>"
  exit 1
fi

WS=/root/thesis_social_navigation_ws
SCEN="$1"; COND="$2"; RUN=$(printf "%02d" "$3"); CFG="$4"
[ -f "$CFG" ] || CFG="$WS/$4"
if [ ! -f "$CFG" ]; then
  echo "config file not found: $4"
  exit 1
fi
STAMP=$(date +%Y%m%d_%H%M%S)
NAME="${SCEN}_${COND}_run${RUN}_${STAMP}"
OUT="$WS/bags/$NAME"
mkdir -p "$OUT/params"

# --- provenance -------------------------------------------------------
# Parameters as the nodes are ACTUALLY running them, dumped live.
for n in /turtlebot4/global_costmap/global_costmap \
         /turtlebot4/local_costmap/local_costmap \
         /turtlebot4/controller_server \
         /turtlebot4/planner_server \
         /turtlebot4/bt_navigator \
         /turtlebot4/velocity_smoother \
         /turtlebot4/amcl \
         /turtlebot4/collision_monitor \
         /camera_ray_person_node \
         /camera_ray_identity_node \
         /human_kf_predictor \
         /predicted_person_cloud_node; do
  f="$OUT/params/$(echo "$n" | tr '/' '_' | sed 's/^_//').yaml"
  # 5 Oct 2026: dumps run in parallel (they took about 30 s one after another)
  ( timeout 15 ros2 param dump "$n" > "$f" 2>/dev/null || {
    echo "WARN: could not dump params of $n"; rm -f "$f"; } ) &
done
ros2 node list > "$OUT/node_list.txt" 2>/dev/null &
ros2 topic list -t > "$OUT/topic_list.txt" 2>/dev/null &
wait

cp "$CFG" "$OUT/"

mkdir -p "$OUT/scripts"
for f in yolo_leg_detector_lidar.py camera_ray_person_node.py \
         camera_ray_identity_node.py human_kf_predictor_lidar.py \
         predicted_person_cloud_node_lidar.py \
         predicted_person_cloud_node_lidar_lane.py \
         social_group_detector_node.py social_group_detector_node_lidarhold.py \
         social_zone_costmap_node.py social_zone_speed_limiter.py \
         social_group_detector_node_lidarhold_simport.py social_zone_costmap_node_simport.py \
         social_group_cloud_node.py beep_retry_node.py \
         blocked_person_beep_node.py blocked_person_beep_node_front.py \
         human_kf_predictor_lidar_noghost.py \
         identity_fusion_node_lidar.py; do
  [ -f "$WS/scripts/$f" ] && cp "$WS/scripts/$f" "$OUT/scripts/"
done
# Lane port: keep the critic source, behaviour trees and launch files of the run.
mkdir -p "$OUT/src"
cp -r "$WS/src/social_critic" "$WS/src/social_critic_lane" "$OUT/src/" 2>/dev/null || true
cp -r "$WS/behavior_trees" "$OUT/" 2>/dev/null || true
cp "$WS"/launch/perception_camray*.launch.py "$WS"/launch/costmap_filter_info*.launch.py "$OUT/" 2>/dev/null || true
cp "$WS"/launch/lab2_bringup*.launch.py "$WS"/launch/nav2_cm.launch.py "$WS"/launch/navigation_cm_launch.py "$OUT/" 2>/dev/null || true
ps -eo args | grep -E "ros2 launch|scripts/.*\.py" | grep -v grep > "$OUT/running_commands.txt" 2>/dev/null || true
md5sum "$WS"/install/social_critic*/lib/*.so > "$OUT/md5sums_critic_lib.txt" 2>/dev/null || true
md5sum "$OUT"/scripts/*.py "$OUT"/*.yaml > "$OUT/md5sums.txt" 2>/dev/null || true

{
  echo "scenario: $SCEN"
  echo "condition: $COND"
  echo "run: $RUN"
  echo "recorded: $STAMP"
  echo "nav2_config: $(basename "$CFG")"
  echo "git: $(cd "$WS" && git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "dirty: $(cd "$WS" && git status --porcelain 2>/dev/null | wc -l) file(s)"
} > "$OUT/run_info.txt"

# --- the bag ----------------------------------------------------------
# Only the topics the analysis scripts read (quick_bag_check.py,
# headon_metrics.py, pair_pass_analysis.py, pair_pass_hwday.py,
# hold_test_hwday.py, goal / lane / hold checks), plus /rosout for the
# critic's lane log lines and /parameter_events for the BlockedHold switching.
TOPICS=(
  # robot, TF, localisation
  /turtlebot4/scan /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/odom
  /turtlebot4/map /turtlebot4/amcl_pose /turtlebot4/hazard_detection /turtlebot4/cmd_audio
  # velocity chain: controller -> smoother -> collision monitor -> robot
  /turtlebot4/cmd_vel_nav /turtlebot4/cmd_vel_smoothed /turtlebot4/cmd_vel
  # Nav2 goal, plan, costmaps (the 2D costmaps only, no voxel grids / costmap_raw)
  /turtlebot4/goal_pose /turtlebot4/plan
  /turtlebot4/navigate_to_pose/_action/status /turtlebot4/navigate_to_pose/_action/feedback
  /turtlebot4/global_costmap/costmap /turtlebot4/local_costmap/costmap
  /turtlebot4/local_costmap/published_footprint
  # perception, groups, zone
  /person_positions_map /camera_ray_clusters /person_positions_fused
  /predicted_person_positions /predicted_person_cloud
  /social_groups /social_zone_map
  # SocialLaneCritic, blocked beep / hold, logs
  /social_critic/lane_block /social_critic/lane_markers
  /blocked_person_event /parameter_events /rosout
)
if [ "${CAMERA:-0}" = "1" ]; then
  TOPICS+=(/turtlebot4/oakd/rgb/preview/image_raw/compressed /turtlebot4/oakd/rgb/preview/camera_info)
  echo "camera preview: recorded (CAMERA=1)"
else
  echo "camera preview: NOT recorded (default; CAMERA=1 to record it)"
fi
echo "camera: ${CAMERA:-0}" >> "$OUT/run_info.txt"
echo "recording $NAME (${#TOPICS[@]} topics) - Ctrl-C to stop"
trap : INT
ros2 bag record -o "$OUT/bag" --include-hidden-topics "${TOPICS[@]}"
trap - INT

echo
python3 "$WS/scripts/quick_bag_check.py" "$OUT/bag" | tee "$OUT/quick_check.txt"
