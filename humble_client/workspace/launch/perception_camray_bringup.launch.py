from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

WS = "/root/thesis_social_navigation_ws"

# CAMERA-PRIMARY pipeline. Replaces lidar_person_detector +
# identity_fusion_node_lidar with camera_ray_person_node +
# camera_ray_identity_node. Downstream (marker, KF, cloud) unchanged.
# The original perception_bringup.launch.py is untouched for ablation.


def generate_launch_description():
    simtime = LaunchConfiguration("use_sim_time")
    coast = LaunchConfiguration("coast_timeout")
    ray_coast = LaunchConfiguration("ray_coast")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("coast_timeout", default_value="1.5"),
        DeclareLaunchArgument("ray_coast", default_value="false"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/yolo_leg_detector_lidar.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-r", "/oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed"],
            name="yolo_leg_detector_lidar", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/camera_ray_person_node.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-p", "scan_topic:=/turtlebot4/scan",
                 "-p", "camera_yaw_offset_deg:=2.25",
                 "-p", "camera_fx:=287.31",
                 "-p", "ray_half_width_deg:=6.0",
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static",
                 "-r", "/map:=/turtlebot4/map"],
            name="camera_ray_person_node", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/camera_ray_identity_node.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-p", ["coast_enable:=", ray_coast],
                 "-p", "scan_topic:=/turtlebot4/scan",
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static"],
            name="camera_ray_identity_node", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/person_marker_publisher.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-p", "lidar_topic:=/camera_ray_clusters",
                 "-p", "output_topic:=/turtlebot4/person_markers"],
            name="person_marker_publisher", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/human_kf_predictor_lidar.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-r", "/odom:=/turtlebot4/odom",
                 "-p", ["coast_timeout:=", coast]],
            name="human_kf_predictor_lidar", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/predicted_person_cloud_node_lidar.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static",
                 "-r", "/map:=/turtlebot4/map"],
            name="predicted_person_cloud_node_lidar", output="screen"),
    ])
