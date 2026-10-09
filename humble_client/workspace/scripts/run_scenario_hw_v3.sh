#!/usr/bin/env bash
# run_scenario_hw_v3.sh - copy of run_headon_hw_v3.sh (unchanged there), 9 Oct 2026: scenario name from SCEN
#   (default headon), e.g. SCEN=pairwide for the standing pair in the wide hallway. Bags: bags/<SCEN>_<cond>_runNN_*.
#   For SCEN=pair*: before recording, scripts/check_group.py must see a "conversation" group for 3 s (robot beeps
#   two high notes); otherwise no run is started. GROUP_WAIT (default 90 s) is how long it waits; GROUP_CHECK=0 skips it.
# run_headon_hw_v3.sh - copy of run_headon_hw_v2.sh (unchanged there), 9 Oct 2026: before recording it runs
#   scripts/check_localization.py --fix, checks again, and does not start the run unless the heading is within 1 deg
#   and the position within 0.10 m (runs noghost 01/02 and rec20 01-bis went to the wrong side with AMCL 2-3.5 deg off).
# run_headon_hw_v2.sh - copy of run_headon_hw.sh (unchanged there), 9 Oct 2026, with two guards against accidental starts:
#   1. refuses a condition + run number that already has a (non-VOID) bag, and prints the next free number;
#   2. Ctrl+C stops the recorder in the container, cancels any Nav2 goal, and renames the bag folder VOID_... with VOID.txt.
# run_headon_hw.sh - one recorded head-on run, driven from the laptop (host side).
# 8 Oct 2026. Rebuilt from run_one.sh of the 5 Oct hardware day (that copy was
# only in a temporary folder), cut down to the head-on scenario, using
# record_hw_run_v4.sh and nav_goal_wait.py instead of the Nav2 launch log.
#
# The stacks (Nav2, perception, ...) must already be running in the container.
#
# USAGE (on the laptop, not in the container):
#   scripts/run_headon_hw.sh <condition> <run_no> [delay_s]
#   scripts/run_headon_hw.sh v4front 4 5
#
# Optional environment:
#   CFG=config/...yaml     Nav2 config to note in the bag (default: approach_hw, today's head-on config)
#   CAMERA=1               also record the camera preview (+28 MB/run)
#   GX GY GQZ GQW          goal (default: far end -12.20, 17.75, facing down the hallway)
#   TIMEOUT=120            seconds to wait for the Nav2 result
#   BEEP=1                 play a start warble before the goal (off by default since 8 Oct 14:21,
#                          Brian: audible but too loud; the walker starts when the robot moves)
#
# Steps: start the recorder -> wait until the bag is written -> wait delay_s ->
# camera check (restart once if silent, no goal if still silent) -> start tone ->
# goal and wait for the result -> stop the recorder (quick check runs) -> metrics.

COND=$1; RUN=$2; DELAY=${3:-5}
[ -n "$COND" ] && [ -n "$RUN" ] || { sed -n 2,20p "$0"; exit 1; }
SCEN=${SCEN:-headon}
CFG=${CFG:-config/social_nav2_headon_F_hwreq_block_approach_hw.yaml}
GX=${GX:--12.20}; GY=${GY:-17.75}; GQZ=${GQZ:-0.943}; GQW=${GQW:-0.334}
cd "$(dirname "$0")/.." || exit 1
C=thesis_social_nav_real
SRC='source /opt/ros/humble/setup.bash; cd /root/thesis_social_navigation_ws; source install/setup.bash'
RN=$(printf "%02d" "$RUN")

[ -f "$CFG" ] || { echo "config not found: $CFG"; exit 1; }
if docker exec $C pgrep -f "[r]os2 bag record" >/dev/null; then
  echo "a recorder is already running - stop it first"; exit 1
fi
# V2 guard 1: this condition + run number must not have a bag yet
if ls -d bags/${SCEN}_${COND}_run${RN}_* >/dev/null 2>&1; then
  n=$RUN; while ls -d bags/${SCEN}_${COND}_run$(printf "%02d" $n)_* >/dev/null 2>&1; do n=$((n + 1)); done
  echo "run $RUN of $COND already exists ($(ls -d bags/${SCEN}_${COND}_run${RN}_* | head -1)) - next free run number: $n"; exit 1
fi
# V3: localization check (fix once, then it must be OK)
LOC="python3 scripts/check_localization.py --ros-args -r /tf:=/turtlebot4/tf -r /tf_static:=/turtlebot4/tf_static"
docker exec $C bash -c "$SRC; ${LOC/check_localization.py/check_localization.py --fix}" 2>&1 | grep -E "^(scan fit|a walker|OK|NOT OK|initialpose)"
sleep 3
if ! docker exec $C bash -c "$SRC; $LOC" 2>&1 | grep -E "^(scan fit|OK|NOT OK)" | tee /dev/stderr | grep -qx "OK"; then
  echo "LOCALIZATION NOT OK after one fix - no run started. Drive or turn the robot a little, or set the pose in RViz, then try again."
  exit 3
fi
echo "localization OK $(date +%H:%M:%S)"
# pair scenarios: the group must be detected before the run
case "$SCEN" in
  pair*)
    if [ "${GROUP_CHECK:-1}" = "1" ]; then
      echo "waiting for the pair to be detected as a conversation (up to ${GROUP_WAIT:-90} s; the robot beeps the result)"
      docker exec $C bash -c "$SRC; python3 scripts/check_group.py --wait ${GROUP_WAIT:-90}"; gr=$?
      if [ $gr -ne 0 ]; then
        echo "NO CONVERSATION GROUP (check_group exit $gr) - no run started. Stand facing each other in camera view and try again."
        exit 4
      fi
    fi ;;
esac
# V2 guard 2: Ctrl+C cleans up
d=""
abort() {
  echo; echo "INTERRUPTED $(date +%H:%M:%S): cancelling Nav2 goals, stopping the recorder"
  docker exec $C bash -c "source /opt/ros/humble/setup.bash; pkill -INT -f '[n]av_goal_wait'; timeout 8 ros2 service call /turtlebot4/navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal '{}' >/dev/null 2>&1; pkill -INT -f '[r]os2 bag record'"
  sleep 4
  [ -z "$d" ] && d=$(ls -dt bags/${SCEN}_${COND}_run${RN}_* 2>/dev/null | head -1)
  if [ -n "$d" ]; then
    b=$(basename "$d")
    docker exec $C bash -c "cd /root/thesis_social_navigation_ws/bags && mv $b VOID_$b && echo 'Interrupted with Ctrl+C at $(date +%H:%M:%S); run_headon_hw_v2.sh stopped the recorder and cancelled goals. Not a run.' > VOID_$b/VOID.txt && chown -R 1000:1000 VOID_$b"
    echo "bag folder renamed bags/VOID_$b"
  fi
  rm -f "$OUTF"; exit 130
}
trap abort INT

OUTF=$(mktemp)
docker exec -e CAMERA=${CAMERA:-0} $C bash -c "$SRC; ./scripts/record_hw_run_v4.sh $SCEN $COND $RUN $CFG 2>&1 | grep -v 'unknown type'" > "$OUTF" 2>&1 &
REC=$!
for i in $(seq 1 80); do
  d=$(ls -dt bags/${SCEN}_${COND}_run${RN}_* 2>/dev/null | head -1)
  if [ -n "$d" ] && ls $d/bag/*.db3 >/dev/null 2>&1; then
    s1=$(du -sk $d/bag | cut -f1); sleep 1; s2=$(du -sk $d/bag | cut -f1)
    [ "$s2" -gt "$s1" ] && { echo "WRITING $d $(date +%H:%M:%S)"; break; }
  fi
  sleep 1
done
if [ -z "$d" ] || ! ls $d/bag/*.db3 >/dev/null 2>&1; then
  echo "bag did not start - recorder output:"; cat "$OUTF"
  docker exec $C pkill -INT -f "[r]os2 bag record"; wait $REC; rm -f "$OUTF"; exit 2
fi

sleep "$DELAY"
# the camera must be publishing right before the goal; restart it once if not
CAM=/turtlebot4/oakd/rgb/preview/image_raw/compressed
camhz() { docker exec $C bash -c "source /opt/ros/humble/setup.bash; timeout 3 ros2 topic hz $CAM 2>&1 | grep -m1 'average rate'"; }
r=$(camhz)
if [ -z "$r" ]; then
  echo "CAMERA not publishing before the goal - restarting it $(date +%H:%M:%S)"
  docker exec $C bash -c "source /opt/ros/humble/setup.bash; timeout 25 ros2 service call /turtlebot4/oakd/stop_camera std_srvs/srv/Trigger >/dev/null 2>&1; sleep 2; timeout 25 ros2 service call /turtlebot4/oakd/start_camera std_srvs/srv/Trigger >/dev/null 2>&1"
  sleep 5; r=$(camhz)
  if [ -z "$r" ]; then
    echo "CAMERA STILL NOT PUBLISHING - no goal sent, recording stopped"
    docker exec $C pkill -INT -f "[r]os2 bag record"; wait $REC; rm -f "$OUTF"; exit 3
  fi
fi
echo "camera before goal: $r"
echo "before goal, /person_positions_fused: $(docker exec $C bash -c "source /opt/ros/humble/setup.bash; timeout 3 ros2 topic echo --once --full-length /person_positions_fused 2>&1 | head -1 | cut -c1-200")"

# start signal for the walker, who cannot see the laptop: a two-pitch warble, about 2 s
# (8 Oct: a single 0.9 s tone was not noticed from the far end; no silent notes,
#  because notes with frequency 0 silenced the whole message on 5 Oct)
if [ "${BEEP:-0}" = "1" ]; then
  docker exec $C bash -c "source /opt/ros/humble/setup.bash; timeout 8 ros2 topic pub --once /turtlebot4/cmd_audio irobot_create_msgs/msg/AudioNoteVector '{append: false, notes: [{frequency: 880, max_runtime: {sec: 0, nanosec: 350000000}}, {frequency: 1320, max_runtime: {sec: 0, nanosec: 350000000}}, {frequency: 880, max_runtime: {sec: 0, nanosec: 350000000}}, {frequency: 1320, max_runtime: {sec: 1, nanosec: 0}}]}'" >/dev/null 2>&1
  sleep 2
fi

docker exec $C bash -c "$SRC; python3 scripts/nav_goal_wait.py --timeout ${TIMEOUT:-120} -- $GX $GY $GQZ $GQW" 2>&1 | grep -E "^(GOAL|goal ended|RESULT)"
sleep 4
docker exec $C pkill -INT -f "[r]os2 bag record"
wait $REC

echo "--- quick check"
sed -n '/^bag:/,$p' "$OUTF" | grep -vE "^  ok  "
echo "--- metrics"
docker exec -e ROS_DOMAIN_ID=77 $C bash -c "$SRC; python3 pytest/analysis/headon_metrics.py $d/bag 2>&1 | grep -v rosbag2_storage > $d/headon_metrics.txt"
grep -E "^(t\+0|first detection|closest approach|detour|stable ids|stops)|robot vx at closest" $d/headon_metrics.txt
sed -n '/nearest LiDAR return/,/per 1 s bin/p' $d/headon_metrics.txt | sed -n 2p
du -sh $d | cut -f1
echo "battery: $(docker exec $C bash -c 'source /opt/ros/humble/setup.bash; timeout 8 ros2 topic echo --once /turtlebot4/battery_state --field percentage 2>&1 | head -1')"
rm -f "$OUTF"
