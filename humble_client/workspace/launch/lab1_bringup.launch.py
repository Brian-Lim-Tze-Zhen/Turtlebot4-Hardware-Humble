#!/usr/bin/env python3
"""
social_nav_bringup.launch.py

Launches the three standard TurtleBot4 stacks together:
  1. localization (AMCL against a saved map)
  2. nav2         (planner/controller/bt_navigator)
  3. rviz         (view_robot)

All three share the same namespace so their topics and TF line up.

Usage (defaults match the manual commands):
    ros2 launch social_nav_bringup.launch.py

Override map or namespace:
    ros2 launch social_nav_bringup.launch.py \
        map:=/root/thesis_social_navigation_ws/other_map.yaml \
        namespace:=/turtlebot4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Arguments
    # ------------------------------------------------------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/root/thesis_social_navigation_ws/map_name.yaml',
        description='Full path to the map yaml file',
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/turtlebot4',
        description='Robot namespace',
    )

    map_yaml = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')

    nav_share = FindPackageShare('turtlebot4_navigation')
    viz_share = FindPackageShare('turtlebot4_viz')

    # ------------------------------------------------------------------
    # 1. Localization (AMCL + map_server)
    # ------------------------------------------------------------------
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_share, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'map': map_yaml,
            'namespace': namespace,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. Nav2
    #
    # Delayed so map_server and AMCL are up first. Nav2's costmaps need
    # the map topic and the map->odom transform at configure time; if
    # they start together, the costmaps log "map frame does not exist"
    # and the lifecycle manager can fail its activation sequence.
    # ------------------------------------------------------------------
    nav2 = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([nav_share, 'launch', 'nav2.launch.py'])
                ),
                launch_arguments={
                    'namespace': namespace,
                }.items(),
            )
        ],
    )

    # ------------------------------------------------------------------
    # 3. RViz
    #
    # Started last so the displays find their topics already advertised
    # rather than showing errors until the stacks come up.
    # ------------------------------------------------------------------
    rviz = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([viz_share, 'launch', 'view_robot.launch.py'])
                ),
                launch_arguments={
                    'namespace': namespace,
                }.items(),
            )
        ],
    )

    return LaunchDescription([
        map_arg,
        namespace_arg,
        localization,
        nav2,
        rviz,
    ])
