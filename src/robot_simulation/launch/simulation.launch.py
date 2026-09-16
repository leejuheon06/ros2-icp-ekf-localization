import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # -----------------------------
    # Package paths
    # -----------------------------

    description_pkg = get_package_share_directory(
        'robot_description'
    )

    simulation_pkg = get_package_share_directory(
        'robot_simulation'
    )

    ros_gz_sim_pkg = get_package_share_directory(
        'ros_gz_sim'
    )


    # -----------------------------
    # URDF / Xacro
    # -----------------------------

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'amr.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        xacro_file
    )

    robot_description = (
        robot_description_config.toxml()
    )


    # -----------------------------
    # World Launch Argument
    # -----------------------------

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty_world.sdf',
        description='Gazebo world file name'
    )

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Robot initial x position'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Robot initial y position'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.10',
        description='Robot initial z position'
    )

    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Robot initial yaw angle [rad]'
    )


    # -----------------------------
    # World
    # -----------------------------

    world_file = PathJoinSubstitution([
        simulation_pkg,
        'worlds',
        LaunchConfiguration('world')
    ])


    # -----------------------------
    # Start Gazebo
    # -----------------------------

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(
            os.path.join(
                ros_gz_sim_pkg,
                'launch',
                'gz_sim.launch.py'
            )
        ),

        launch_arguments={
            'gz_args': [
                '-r ',
                world_file
            ]
        }.items()

    )


    # -----------------------------
    # Robot State Publisher
    # -----------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{

            'robot_description':
                robot_description,

            'use_sim_time':
                True
        }]
    )


    # -----------------------------
    # Spawn Robot
    # -----------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[

            '-name',
            'icp_ekf_amr',

            '-topic',
            'robot_description',

            '-x',
            LaunchConfiguration('x'),

            '-y',
            LaunchConfiguration('y'),

            '-z',
            LaunchConfiguration('z'),

            '-Y',
            LaunchConfiguration('yaw')

        ],
        output='screen'
    )


    # -----------------------------
    # ROS2 <-> Gazebo Bridge
    # -----------------------------

    bridge_config = os.path.join(
        simulation_pkg,
        'config',
        'bridge.yaml'
    )


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            'config_file:=' + bridge_config
        ],
        output='screen'
    )


    # -----------------------------
    # Launch Description
    # -----------------------------

    return LaunchDescription([

        world_arg,

        x_arg,
        y_arg,
        z_arg,
        yaw_arg,

        gazebo,

        robot_state_publisher,

        spawn_robot,

        bridge

    ])