from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_simulation'),
                'launch',
                'localization_icp.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
            'map': map_yaml,
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    delayed_navigation = TimerAction(
        period=3.0,
        actions=[navigation]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='localization_world.sdf'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='-3.8'),
        DeclareLaunchArgument('z', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),

        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'ros2_icp_ekf_localization',
                'maps',
                'localization_map.yaml'
            ])
        ),

        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_simulation'),
                'config',
                'nav2_icp_params.yaml'
            ])
        ),

        localization,
        delayed_navigation,
    ])
