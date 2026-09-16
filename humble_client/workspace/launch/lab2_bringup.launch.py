from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/root/thesis_social_navigation_ws/lab_map_name.yaml'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/turtlebot4'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/root/thesis_social_navigation_ws/config/nav2_params.yaml'
    )

    # THESIS ADDITION: /turtlebot4/odom is BEST_EFFORT, but
    # bt_navigator and controller_server both declare
    # odom_topic: odom_reliable. That topic is produced by this relay
    # and by nothing else - when the relay is not running the topic
    # does not exist, both nodes subscribe to nothing, and MPPI runs
    # with no odometry at all. This was the state for an unknown number
    # of trials (found 8 Sep 2026). Nav2 subscribes once at startup, so
    # the relay must be up BEFORE nav2 - hence the TimerAction below.
    odom_relay = ExecuteProcess(
        cmd=['python3', '/root/thesis_social_navigation_ws/odom_qos_relay.py'],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'localization.launch.py'
            ])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'namespace': LaunchConfiguration('namespace'),
            'params': '/root/thesis_social_navigation_ws/config/localization.yaml',
            'use_sim_time': 'false',
        }.items()
    )
    
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    view_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_robot.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )

    return LaunchDescription([
        map_arg,
        namespace_arg,
        params_file_arg,
        odom_relay,
        localization,
        # Delay so /turtlebot4/odom_reliable exists before
        # controller_server and bt_navigator subscribe to it.
        TimerAction(period=3.0, actions=[nav2]),
        view_robot,
    ])
