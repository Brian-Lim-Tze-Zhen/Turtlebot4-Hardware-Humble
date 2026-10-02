#!/usr/bin/env bash
# replay_camera_ray.sh - replay recorded bags through camera_ray_person_node
# (old vs new script) with no robot, and save the node logs for comparison.
#
# Run INSIDE the container. Uses its own ROS domain so a live robot or
# another replay cannot contaminate TF (TF_OLD_DATA).
#
#   ./scripts/replay_camera_ray.sh            # all bags below, old + new
#   ./scripts/replay_camera_ray.sh new        # new script only
#
# Logs land in bags/replay_logs/<bag>__<old|new>.log
export ROS_DOMAIN_ID=77
WS=/root/thesis_social_navigation_ws
cd "$WS" || exit 1
OLD=bags/headon_F_wz07_infl035_kfq_20261002_145103/camera_ray_person_node.py  # == git HEAD
NEW=scripts/camera_ray_person_node.py
LOGS=bags/replay_logs
mkdir -p "$LOGS"
WHICH="${1:-old new}"
BAGS="
headon_F_wz07_b1200_20261002_134111
headon_F_wz07_pf9b_20261002_141533
headon_F_wz07_infl035_slope12_20261002_143312
headon_F_wz07_infl035_kfq_20261002_145103
headon_F_wz07_infl035_slope12_20261002_143752
proto_C_headon
proto_G_lateral
proto_G2_lateral
proto_K_occlusion
proto_K2_occlusion
proto_K3_occlusion
proto_L_nav_walkpast
proto_L2_nav_walkpast
"
for B in $BAGS; do
  BAGDIR=$(dirname "$(find bags/$B -name metadata.yaml | head -1)")
  for V in $WHICH; do
    [ "$V" = old ] && SCRIPT=$OLD || SCRIPT=$NEW
    LOG="$LOGS/${B}__${V}.log"
    echo "=== $B [$V]"
    python3 "$SCRIPT" --ros-args \
      -p use_sim_time:=true -p scan_topic:=/turtlebot4/scan \
      -p camera_yaw_offset_deg:=2.25 -p camera_fx:=287.31 -p ray_half_width_deg:=6.0 \
      -p output_topic:=/camera_ray_clusters_new \
      -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static -r /map:=/turtlebot4/map \
      > "$LOG" 2>&1 &
    NODE=$!
    sleep 5
    ros2 bag play "$BAGDIR" --clock \
      --qos-profile-overrides-path config/replay_qos.yaml \
      --topics /person_positions_map /turtlebot4/scan /turtlebot4/odom \
               /turtlebot4/tf /turtlebot4/tf_static /turtlebot4/map > /dev/null 2>&1
    sleep 1
    kill -INT $NODE; wait $NODE 2>/dev/null
    echo "    refused $(grep -c refused "$LOG")  motion-release $(grep -c motion-release "$LOG")  width-release $(grep -c ' release ' "$LOG")  TF_OLD_DATA $(grep -c TF_OLD_DATA "$LOG")"
  done
done
echo "done - logs in $LOGS"
