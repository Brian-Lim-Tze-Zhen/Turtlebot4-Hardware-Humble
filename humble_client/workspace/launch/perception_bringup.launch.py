from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

WS = "/root/thesis_social_navigation_ws"


def generate_launch_description():
    span = LaunchConfiguration("confirm_min_span")
    rgate = LaunchConfiguration("range_ratio_gate")
    simtime = LaunchConfiguration("use_sim_time")
    coast = LaunchConfiguration("coast_timeout")

    nodes = [
        DeclareLaunchArgument("confirm_min_span", default_value="1.5"),
        DeclareLaunchArgument("range_ratio_gate", default_value="2.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("coast_timeout", default_value="1.5"),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/lidar_person_detector.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-r", "/map:=/turtlebot4/map",
                "-p", ["confirm_min_span:=", span],
            ],
            name="lidar_person_detector",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/yolo_leg_detector_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed",
            ],
            name="yolo_leg_detector_lidar",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/identity_fusion_node_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-p", ["range_ratio_gate:=", rgate],
            ],
            name="identity_fusion_node_lidar",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/person_marker_publisher.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-p", "output_topic:=/turtlebot4/person_markers",
            ],
            name="person_marker_publisher",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/human_kf_predictor_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/odom:=/turtlebot4/odom",
                "-p", ["coast_timeout:=", coast],
            ],
            name="human_kf_predictor_lidar",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", f"{WS}/scripts/predicted_person_cloud_node_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-r", "/local_costmap/clear_entirely_local_costmap:=/turtlebot4/local_costmap/clear_entirely_local_costmap",
                "-r", "/global_costmap/clear_entirely_global_costmap:=/turtlebot4/global_costmap/clear_entirely_global_costmap",
                "-r", "/local_costmap/clear_around_pose_local_costmap:=/turtlebot4/local_costmap/clear_around_pose_local_costmap",
            ],
            name="predicted_person_cloud_node_lidar",
            output="screen",
        ),
    ]

    return LaunchDescription(nodes)
