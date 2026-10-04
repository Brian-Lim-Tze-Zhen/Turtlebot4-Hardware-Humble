from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

# social_perception_bringup.launch.py
#
# Perception + social-zone stack for the CONVERSATION (ablation F)
# scenarios. Supersedes perception_bringup.launch.py for these runs;
# run one or the other, never both.
#
# WHAT THIS ADDS over perception_bringup.launch.py
#   1. angle_gate_deg 12 -> 18. The 18 Sep histogram of camera<->lidar
#      pair-bearing diffs was a broad unimodal spread peaking at 2-4 deg
#      with a long tail, TRUNCATED at the 12 deg gate (200 samples at
#      11 deg, 11 at 12). That is latency/noise, not a constant
#      extrinsic offset - calibrating an offset would not fix it. The
#      truncation was starving the detector: fresh_bbox=1 was the
#      dominant state, so only ONE of the two people was ever
#      camera-confirmed and the other coasted as a frozen ghost.
#      At 3 m, 18 deg spans 0.94 m - still well inside a 1.4 m pair
#      separation, so the two people remain distinguishable. Above
#      ~20 deg that stops being true.
#
#   2. social_group_detector_node with costmap_topic pointed at the
#      GLOBAL costmap. Its default is the LOCAL costmap, whose
#      global_frame is ODOM and which is a rolling 7x7 m window. Group
#      centroids are in MAP coordinates, so every probe landed outside
#      that window, read unknown, and returned "measured clearance
#      0.00m". The buffer was pinned at the 0.05 floor everywhere, wide
#      hall or tight corridor, and the narrow/wide flip could never
#      fire. The global costmap is in the map frame and covers the
#      whole map, which is what the probe needs.
#
#   3. social_zone_costmap_node - graded OccupancyGrid social zones.
#      Needs no TF remap: zones arrive already in the map frame.
#
#   4. social_zone_speed_limiter - caps speed to 30% while the robot is
#      inside a zone the corridor forced it into.
#
# LAUNCH ORDER
#   The zone node must be publishing BEFORE nav2's social_zone_layer
#   subscribes. StaticLayer BLOCKS the entire costmap update until it
#   has received a map - a silent publisher stops global planning
#   outright, it does not merely contribute no cost. So start this
#   launch first, wait for "Map geometry:" in its output, THEN start
#   lab2_bringup with the ablation F params.
#
#   The speed limiter is delayed because /turtlebot4/speed_limit only
#   matters once controller_server exists; starting it early is
#   harmless but its logs are cleaner this way.
#
# NOT INCLUDED
#   social_group_cloud_node.py (ablation E's lethal pointcloud) - it is
#   superseded by social_zone_costmap_node and the two must not run
#   together.

WS = "/root/thesis_social_navigation_ws"


def generate_launch_description():
    span = LaunchConfiguration("confirm_min_span")
    rgate = LaunchConfiguration("range_ratio_gate")
    agate = LaunchConfiguration("angle_gate_deg")
    simtime = LaunchConfiguration("use_sim_time")
    coast = LaunchConfiguration("coast_timeout")
    probe_costmap = LaunchConfiguration("probe_costmap")
    show_debug = LaunchConfiguration("show_debug_image")

    nodes = [
        DeclareLaunchArgument("confirm_min_span", default_value="1.5"),
        DeclareLaunchArgument("range_ratio_gate", default_value="2.0"),
        # See note 1 above. 12.0 restores the pre-18-Sep behaviour.
        DeclareLaunchArgument("angle_gate_deg", default_value="18.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("coast_timeout", default_value="1.5"),
        # See note 2 above. Must be a MAP-frame costmap.
        DeclareLaunchArgument(
            "probe_costmap",
            default_value="/turtlebot4/global_costmap/costmap"),
        # cv2.imshow needs an X display; the container's GL stack is
        # broken (see rviz libGL errors), and a failing imshow inside
        # the image callback can stall the subscription. Crops are
        # saved to debug_crops/ regardless, which is more useful after
        # the fact anyway.
        DeclareLaunchArgument("show_debug_image", default_value="false"),

        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/lidar_person_detector.py",
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
                "python3", "-u", f"{WS}/scripts/yolo_leg_detector_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/oakd/rgb/preview/image_raw/compressed:=/turtlebot4/oakd/rgb/preview/image_raw/compressed",
            ],
            name="yolo_leg_detector_lidar",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/identity_fusion_node_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-p", ["range_ratio_gate:=", rgate],
                "-p", ["angle_gate_deg:=", agate],
            ],
            name="identity_fusion_node_lidar",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/person_marker_publisher.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-p", "output_topic:=/turtlebot4/person_markers",
            ],
            name="person_marker_publisher",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/human_kf_predictor_lidar.py",
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
                "python3", "-u",
                f"{WS}/scripts/predicted_person_cloud_node_lidar.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-r", "/map:=/turtlebot4/map",
            ],
            name="predicted_person_cloud_node_lidar",
            output="screen",
        ),

        # --- social zone stack (ablation F) ---------------------------
        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/social_group_detector_node.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
                "-r", "/tf:=/turtlebot4/tf",
                "-r", "/tf_static:=/turtlebot4/tf_static",
                "-p", ["costmap_topic:=", probe_costmap],
                "-p", ["show_debug_image:=", show_debug],
            ],
            name="social_group_detector",
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "python3", "-u", f"{WS}/scripts/social_zone_costmap_node.py",
                "--ros-args",
                "-p", ["use_sim_time:=", simtime],
            ],
            name="social_zone_costmap_node",
            output="screen",
        ),
        # Delayed: only meaningful once controller_server is up. Keeps
        # its log free of startup noise.
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    "python3", "-u",
                    f"{WS}/scripts/social_zone_speed_limiter.py",
                    "--ros-args",
                    "-p", ["use_sim_time:=", simtime],
                    "-r", "/tf:=/turtlebot4/tf",
                    "-r", "/tf_static:=/turtlebot4/tf_static",
                ],
                name="social_zone_speed_limiter",
                output="screen",
            ),
        ]),
    ]

    return LaunchDescription(nodes)
