# --------------------------------------------------------------------------------------------------
# What this launch file is for (plain words):
#   Starts everything the robot needs for a recorded run, in one command, EXCEPT the recorder:
#     1. Nav2 with localisation, the odometry relay and RViz   (launch/lab2_bringup_cm.launch.py)
#     2. the zone stack: filter info, zone node, group detector (launch/costmap_filter_info_hwdet_mapprobe.launch.py)
#     3. perception: detector, camera rays, identity, Kalman predictor without ghosts, person cloud
#                                                               (launch/perception_camray_lane_noghost_bringup.launch.py)
#     4. the two nodes that hold the robot for a blocking person and beep
#                                                               (scripts/beep_retry_node.py,
#                                                                scripts/blocked_person_beep_node_front_atgoal.py hold:=true)
#   These are the processes that were running in the counted runs of 9 October 2026 (see
#   running_commands.txt in bags/headon_rec20noghost_run03_20261009_121336); none of those files is changed.
#   The recorder is NOT started: recording stays with scripts/run_headon_hw_v3.sh and
#   scripts/run_scenario_hw_v3.sh, which expect these stacks to be running.
#
#   Written 11 October 2026 for the robot day of 12 October 2026 (group-skip test).
#
# Usage (inside the container thesis_social_nav_real, after the time sync):
#   source /opt/ros/humble/setup.bash
#   cd /root/thesis_social_navigation_ws && source install/setup.bash
#   ros2 launch launch/robot_day_all.launch.py
#
# Arguments (all optional):
#   params_file:=<Nav2 file>      default: the group-skip variant of the final robot configuration
#                                 (config/social_nav2_headon_F_hwreq_block_approach_tf05_rec20_noghost_groupskip_hw.yaml);
#                                 for the configuration of 9 October use ..._tf05_rec20_noghost_hw.yaml
#   map:=<map yaml>               default: map/hall_map_name.yaml
#   zone_params_file:=<file>      default: config/social_nav2_headon_F_hwreq_block_approach_hw.yaml (as on 9 October)
#   zone_delay:=20.0              seconds after Nav2 before the zone stack starts (it needs the map from Nav2)
#   perception_delay:=25.0        seconds after Nav2 before perception starts
#   hold_delay:=30.0              seconds after Nav2 before the two hold nodes start
#   hold:=true                    false leaves the two hold nodes out
#
# After it is up: set the pose in RViz if needed, then check with scripts/check_localization.py
# (the run scripts do this themselves and refuse to start a run if it is off).
# Stop everything with Ctrl+C in this terminal.
# --------------------------------------------------------------------------------------------------
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))
WS = "/root/thesis_social_navigation_ws"
TF = ["-r", "/tf:=/turtlebot4/tf", "-r", "/tf_static:=/turtlebot4/tf_static"]


def include(name, **args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(HERE, name)),
                                    launch_arguments=args.items())


def generate_launch_description():
    params = LaunchConfiguration("params_file")

    nav2 = include("lab2_bringup_cm.launch.py", map=LaunchConfiguration("map"), namespace="/turtlebot4",
                   params_file=params)
    zone = include("costmap_filter_info_hwdet_mapprobe.launch.py", params_file=LaunchConfiguration("zone_params_file"))
    perception = include("perception_camray_lane_noghost_bringup.launch.py")
    beep_retry = ExecuteProcess(
        cmd=["python3", f"{WS}/scripts/beep_retry_node.py", "--ros-args"] + TF,
        name="beep_retry_node", output="screen", condition=IfCondition(LaunchConfiguration("hold")))
    blocked_hold = ExecuteProcess(
        cmd=["python3", f"{WS}/scripts/blocked_person_beep_node_front_atgoal.py", "--ros-args", "-p", "hold:=true"] + TF,
        name="blocked_person_beep_node", output="screen", condition=IfCondition(LaunchConfiguration("hold")))

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=f"{WS}/config/social_nav2_headon_F_hwreq_block_approach_tf05_rec20_noghost_groupskip_hw.yaml"),
        DeclareLaunchArgument("map", default_value=f"{WS}/map/hall_map_name.yaml"),
        DeclareLaunchArgument("zone_params_file",
                              default_value=f"{WS}/config/social_nav2_headon_F_hwreq_block_approach_hw.yaml"),
        DeclareLaunchArgument("zone_delay", default_value="20.0"),
        DeclareLaunchArgument("perception_delay", default_value="25.0"),
        DeclareLaunchArgument("hold_delay", default_value="30.0"),
        DeclareLaunchArgument("hold", default_value="true"),
        LogInfo(msg=["[robot_day_all] Nav2 file: ", params]),
        LogInfo(msg="[robot_day_all] The recorder is NOT started: use scripts/run_headon_hw_v3.sh or run_scenario_hw_v3.sh"),
        nav2,
        TimerAction(period=LaunchConfiguration("zone_delay"),
                    actions=[LogInfo(msg="[robot_day_all] starting the zone stack"), zone]),
        TimerAction(period=LaunchConfiguration("perception_delay"),
                    actions=[LogInfo(msg="[robot_day_all] starting perception"), perception]),
        TimerAction(period=LaunchConfiguration("hold_delay"),
                    actions=[LogInfo(msg="[robot_day_all] starting the hold nodes"), beep_retry, blocked_hold]),
    ])
