#!/usr/bin/env bash
# record_social_bag.sh — record one social-zone scenario with provenance.
#
# WHY A WRAPPER: hardware access ends soon and every remaining analysis
# is offline. A bag without the config and node versions that produced
# it is unreplayable six weeks later — you cannot tell whether a zone
# was graded or lethal, or which buffer the detector was using. This
# captures all of that alongside the bag, same convention as the
# headon_real_* bags.
#
# USAGE (one scenario per invocation):
#   ./record_social_bag.sh F_narrow_pass
#   ./record_social_bag.sh E_narrow_control
#
# Ctrl-C to stop the recording. Everything lands in
# bags/<name>_<timestamp>/.

set -euo pipefail

if [ $# -lt 1 ]; then
  echo "usage: $0 <scenario_name>"
  exit 1
fi

WS=/root/thesis_social_navigation_ws
NAME="$1"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$WS/bags/${NAME}_${STAMP}"

mkdir -p "$OUT"

# --- provenance -------------------------------------------------------
# Which params Nav2 is ACTUALLY running, not which file you think you
# launched. Dumped live from the running nodes.
ros2 param dump /turtlebot4/global_costmap/global_costmap \
  > "$OUT/global_costmap_params.yaml" 2>/dev/null || \
  echo "WARN: could not dump global_costmap params"

ros2 param dump /turtlebot4/controller_server \
  > "$OUT/controller_server_params.yaml" 2>/dev/null || \
  echo "WARN: could not dump controller_server params"

# Exact node source that produced this run.
for f in social_group_detector_node.py social_group_detector_node_lidarhold.py social_group_cloud_node.py camera_ray_person_node.py camera_ray_identity_node.py yolo_leg_detector_lidar.py beep_retry_node.py \
         social_zone_costmap_node.py \
         social_zone_speed_limiter.py \
         predicted_person_cloud_node_lidar.py \
         identity_fusion_node_lidar.py \
         human_kf_predictor_lidar.py; do
  [ -f "$WS/scripts/$f" ] && cp "$WS/scripts/$f" "$OUT/" || true
done

md5sum "$OUT"/*.py > "$OUT/md5sums.txt" 2>/dev/null || true

{
  echo "scenario: $NAME"
  echo "recorded: $STAMP"
  echo "git: $(cd "$WS" && git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "dirty: $(cd "$WS" && git status --porcelain 2>/dev/null | wc -l) file(s)"
} > "$OUT/run_info.txt"

# --- the bag ----------------------------------------------------------
# Inputs (re-derive zones offline), outputs under test, and behaviour
# ground truth. See the notes in this file's header for why costmap is
# included despite its size.
ros2 bag record -o "$OUT/bag" -a --include-hidden-topics -x ".*(image|compressed|/oakd/).*"
