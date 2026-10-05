from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# HARDWARE DETECTOR + MAP PROBE (5 Oct 2026): third variant. The HARDWARE group
# detector (scripts/social_group_detector_node_lidarhold.py, unchanged, with its
# 1 Oct LiDAR hold) is started with costmap_topic = the STATIC MAP, so its
# wide/narrow test measures to the mapped walls as the simulation does, not on
# the live global costmap. The zone node is the simulation one
# (scripts/social_zone_costmap_node_simport.py). Reason: with the ported
# simulation detector the pair was dropped within seconds of the robot moving
# ("LIDAR sees anchor B vacant", bags pairwide_simport_run01/02 of 5 Oct), and
# with the hardware detector on the global costmap a pair in the 3.6-4.0 m
# hallway was classified narrow (bag pairwide_zone_run01 of 5 Oct).
#
# Text below is from the simulation-port variant it was copied from:
# SIM PORT (5 Oct 2026): copy of costmap_filter_info.launch.py that starts the
# SIMULATION versions of the zone node and the group detector
# (scripts/social_zone_costmap_node_simport.py,
#  scripts/social_group_detector_node_lidarhold_simport.py), with the
# simulation's settings: the detector probes the STATIC MAP for the
# wide/narrow test (costmap_topic = the map, as in run_conv_F_trial.sh), not
# the global costmap. queue_detection stays off as in the conversation trials.
#
# ABLATION F: publishes the CostmapFilterInfo that KeepoutFilter needs
# in order to interpret /social_zone_map. Without this the filter loads,
# logs that it is waiting for filter info, and applies nothing.
#
# The MASK itself comes from social_zone_costmap_node.py (latched,
# TRANSIENT_LOCAL) - NOT from a map_server, unlike the Nav2 keepout
# tutorial, because the zone is generated per episode from perception
# rather than annotated on a map file. Only the info server is needed.
#
# Both nodes are lifecycle-managed: costmap_filter_info_server is a
# lifecycle node and stays unconfigured (publishing nothing) unless a
# lifecycle manager brings it up, hence the manager below.
#
# 1 Oct 2026: social_zone_costmap_node.py added here, so the filter info
# and the mask it describes always start together. Wait for
# "Map geometry:" before launching Nav2. zone_node:=false starts the
# info server alone (e.g. to run the zone node in its own terminal).
#
# 1 Oct 2026: social_group_detector_node_lidarhold.py added too, so the
# whole ablation-F social stack (detector -> zone node -> filter info)
# is one command. Probes the GLOBAL costmap (map frame) with the debug
# window off. detector:=false leaves it out.

WS = "/root/thesis_social_navigation_ws"
PARAMS = (f"{WS}/config/"
          "social_nav2_ablation_F_socialzone_keepout.yaml")


def generate_launch_description():
    simtime = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    zone_node = LaunchConfiguration("zone_node")
    detector = LaunchConfiguration("detector")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=PARAMS),
        DeclareLaunchArgument("zone_node", default_value="true"),
        DeclareLaunchArgument("detector", default_value="true"),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            emulate_tty=True,
            parameters=[params, {"use_sim_time": simtime}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_costmap_filters",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "use_sim_time": simtime,
                "autostart": True,
                "node_names": ["costmap_filter_info_server"],
            }],
        ),
        ExecuteProcess(
            cmd=["python3", "-u", f"{WS}/scripts/social_zone_costmap_node_simport.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-p", "map_topic:=/turtlebot4/map"],
            name="social_zone_costmap_node",
            output="screen",
            condition=IfCondition(zone_node),
        ),
        ExecuteProcess(
            cmd=["python3", "-u",
                 f"{WS}/scripts/social_group_detector_node_lidarhold.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static",
                 "-p", "costmap_topic:=/turtlebot4/map",
                 "-p", "show_debug_image:=false"],
            name="social_group_detector",
            output="screen",
            condition=IfCondition(detector),
        ),
    ])
