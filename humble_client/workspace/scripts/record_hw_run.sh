#!/usr/bin/env bash
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
#   ./record_hw_run.sh <scenario> <condition> <run_no> <nav2_config.yaml>
#   ./record_hw_run.sh headon cloudon 1 config/social_nav2_ablation_F_socialzone_keepout.yaml
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
         identity_fusion_node_lidar.py; do
  [ -f "$WS/scripts/$f" ] && cp "$WS/scripts/$f" "$OUT/scripts/"
done
# Lane port: keep the critic source, behaviour trees and launch files of the run.
mkdir -p "$OUT/src"
cp -r "$WS/src/social_critic" "$WS/src/social_critic_lane" "$OUT/src/" 2>/dev/null || true
cp -r "$WS/behavior_trees" "$OUT/" 2>/dev/null || true
cp "$WS"/launch/perception_camray*.launch.py "$WS"/launch/costmap_filter_info*.launch.py "$OUT/" 2>/dev/null || true
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
# Everything except image streams, but KEEP the compressed camera preview
# and its camera_info (needed to re-run YOLO and the pair classifier).
# /trajectories (MPPI rollout markers) is left out: 27 MB/s, not used by
# any analysis script.
KEEP='/turtlebot4/oakd/rgb/preview/(image_raw/compressed|camera_info)'
echo "recording $NAME - Ctrl-C to stop"
trap : INT
ros2 bag record -o "$OUT/bag" -a --include-hidden-topics \
  -x "^(?!${KEEP}\$).*(image|compressed|/oakd/|^/trajectories\$).*"
trap - INT

echo
python3 "$WS/scripts/quick_bag_check.py" "$OUT/bag" | tee "$OUT/quick_check.txt"
