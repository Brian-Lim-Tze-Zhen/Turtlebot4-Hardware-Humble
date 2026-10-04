#!/bin/bash
# compare_detectors.sh
#
# Runs the BUGGY and FIXED lidar_person_detector.py against a recorded bag
# and counts how many confirmed tracks (travel > 0.5m) get dropped.
#
# Requires ROS 2 Humble + the hardware workspace sourced.
# Usage: bash compare_detectors.sh <bag_dir>
#
BAG="${1:-/home/brian/Desktop/Turtlebot4/humble_client/workspace/bags/berth06_20260910_111228}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUGGED_DETECTOR="$BAG/lidar_person_detector.py"
FIXED_DETECTOR="$SCRIPT_DIR/lidar_person_detector_fixed.py"
LOG_DIR="$SCRIPT_DIR/compare_logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null

echo "=============================================="
echo " Detector comparison: static_streak bug"
echo " Bag: $BAG"
echo "=============================================="

run_pass() {
    local label="$1"
    local detector="$2"
    local log="$LOG_DIR/${label}.log"
    local cluster_log="$LOG_DIR/${label}_clusters.log"

    echo ""
    echo "--- $label ---"
    echo "    Detector: $detector"

    # Subscribe to /lidar_person_clusters
    ros2 topic echo /lidar_person_clusters std_msgs/msg/String \
        --no-daemon > "$cluster_log" 2>/dev/null &
    ECHO_PID=$!

    # Start detector
    python3 "$detector" \
        --ros-args \
        -p use_sim_time:=true \
        > "$log" 2>&1 &
    DETECTOR_PID=$!
    sleep 3

    # Play bag
    ros2 bag play "$BAG" \
        --clock \
        --rate 1.0 \
        > /dev/null 2>&1
    sleep 3

    kill "$DETECTOR_PID" 2>/dev/null
    kill "$ECHO_PID" 2>/dev/null
    wait "$DETECTOR_PID" 2>/dev/null || true
    wait "$ECHO_PID" 2>/dev/null || true

    echo "    Detector log: $log"
    echo "    Cluster log:  $cluster_log"
}

run_pass "A_bugged" "$BUGGED_DETECTOR"
run_pass "B_fixed"  "$FIXED_DETECTOR"

echo ""
echo "=============================================="
echo " Parsing cluster logs..."
echo "=============================================="
python3 - "$LOG_DIR/A_bugged_clusters.log" "BUGGED" \
         "$LOG_DIR/B_fixed_clusters.log"  "FIXED" << 'PYEOF'
import sys, re

def analyze(path, label):
    print(f"\n=== {label} ({path}) ===")
    track_first = {}; track_last = {}; track_positions = {}
    prev_ids = set(); t = 0.0
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                for entry in line.split(';'):
                    parts = entry.strip().split(',')
                    if len(parts) < 3:
                        continue
                    try:
                        tid = int(parts[0]); x = float(parts[1]); y = float(parts[2])
                    except ValueError:
                        continue
                    t += 0.0001
                    if tid not in track_first:
                        track_first[tid] = t
                    track_last[tid] = t
                    track_positions.setdefault(tid, []).append((x, y))

                cur = set()
                for e in line.split(';'):
                    p = e.strip().split(',')
                    try: cur.add(int(p[0]))
                    except: pass
                for tid in prev_ids - cur:
                    pos = track_positions.get(tid, [])
                    travel = sum(
                        ((pos[i][0]-pos[i-1][0])**2 + (pos[i][1]-pos[i-1][1])**2)**0.5
                        for i in range(1, len(pos))
                    )
                    if travel > 0.5:
                        lx, ly = pos[-1] if pos else (0,0)
                        lifetime = track_last[tid] - track_first[tid]
                        print(f"  [DROP] id:{tid:2d} travel={travel:.2f}m lifetime={lifetime:.1f}s last=({lx:.2f},{ly:.2f})")
                prev_ids = cur
    except FileNotFoundError:
        print(f"  File not found: {path}")
        return

    confirmed_drops = sum(
        1 for tid in track_positions
        if sum(((track_positions[tid][i][0]-track_positions[tid][i-1][0])**2 +
                (track_positions[tid][i][1]-track_positions[tid][i-1][1])**2)**0.5
               for i in range(1, len(track_positions[tid]))) > 0.5
        and tid not in prev_ids
    )
    print(f"  Confirmed track drops: {confirmed_drops}")
    print(f"  Total tracks seen:     {len(track_first)}")

analyze(sys.argv[1], sys.argv[2])
analyze(sys.argv[3], sys.argv[4])
PYEOF

echo ""
echo "Full logs: $LOG_DIR/"
