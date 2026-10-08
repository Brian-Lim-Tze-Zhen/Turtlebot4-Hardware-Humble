from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

WS = "/root/thesis_social_navigation_ws"

# NOGHOST (8 Oct 2026): copy of perception_camray_lane_bringup.launch.py (kept
# unchanged), identical except that the KF predictor is
# scripts/human_kf_predictor_lidar_noghost.py: a coasting track whose coasted
# position comes within 1.0 m of the robot is dropped (bag
# headon_v4front_run03_20261008_112019: the coast ran the passed walker onto
# the robot and the goal aborted). That node needs the robot pose, hence the
# /turtlebot4/tf remaps on it.

# CAMERA-PRIMARY pipeline. Replaces lidar_person_detector +
# identity_fusion_node_lidar with camera_ray_person_node +
# camera_ray_identity_node. Downstream (marker, KF, cloud) unchanged.
# The original perception_bringup.launch.py is untouched for ablation.
#
# LANE PORT (5 Oct 2026): copy of perception_camray_bringup.launch.py for the
# head-on lane rules (config/social_nav2_headon_F_hwreq_block_hw.yaml), with the
# perception settings the simulation uses for its *hwreq* configs:
#   - person cloud = body disk only (ellipse_a_slope 0.0, ellipse_a_max 0.3,
#     ellipse_b 0.15, person_disk_radius 0.30); the forward ellipse is off
#     because the SocialLaneCritic lane rule does that job
#   - ray_coast on (LiDAR follows a person the camera has lost)
# The cloud node is scripts/predicted_person_cloud_node_lidar_lane.py (the
# simulation's version: the hardware node plus these parameters).


def generate_launch_description():
    simtime = LaunchConfiguration("use_sim_time")
    coast = LaunchConfiguration("coast_timeout")
    ray_coast = LaunchConfiguration("ray_coast")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("coast_timeout", default_value="1.5"),
        DeclareLaunchArgument("ray_coast", default_value="true"),
        # Stationary hold of the identity node (default on = node default).
        # hold_enable:=false for head-on runs: with the walker standing before
        # the run, the hold kept the old identity at the standing position for
        # about 1 s after the camera lost it, the walker came back with a new
        # identity and the critic froze a second lane on the other side
        # (bag headon_lanepilot_run02_20261005_093344).
        DeclareLaunchArgument("hold_enable", default_value="true"),
        DeclareLaunchArgument("ellipse_a_slope", default_value="0.0"),
        DeclareLaunchArgument("ellipse_a_max", default_value="0.3"),
        DeclareLaunchArgument("ellipse_b", default_value="0.15"),
        DeclareLaunchArgument("person_disk_radius", default_value="0.30"),
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
                 "-p", ["hold_enable:=", LaunchConfiguration("hold_enable")],
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
            cmd=["python3", f"{WS}/scripts/human_kf_predictor_lidar_noghost.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-r", "/odom:=/turtlebot4/odom",
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static",
                 "-p", ["coast_timeout:=", coast]],
            name="human_kf_predictor_lidar", output="screen"),
        ExecuteProcess(
            cmd=["python3", f"{WS}/scripts/predicted_person_cloud_node_lidar_lane.py",
                 "--ros-args", "-p", ["use_sim_time:=", simtime],
                 "-p", ["ellipse_a_slope:=", LaunchConfiguration("ellipse_a_slope")],
                 "-p", ["ellipse_a_max:=", LaunchConfiguration("ellipse_a_max")],
                 "-p", ["ellipse_b:=", LaunchConfiguration("ellipse_b")],
                 "-p", ["person_disk_radius:=", LaunchConfiguration("person_disk_radius")],
                 "-r", "/tf:=/turtlebot4/tf",
                 "-r", "/tf_static:=/turtlebot4/tf_static",
                 "-r", "/map:=/turtlebot4/map"],
            name="predicted_person_cloud_node_lidar", output="screen"),
    ])
