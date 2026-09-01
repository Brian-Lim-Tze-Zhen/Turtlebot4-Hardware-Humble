from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
        localization,
        nav2,
        view_robot,
    ])
