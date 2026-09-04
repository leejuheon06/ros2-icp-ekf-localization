import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    # World
    # -----------------------------

    world_file = os.path.join(
        simulation_pkg,
        'worlds',
        'empty_world.sdf'
    )


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
            'gz_args': '-r ' + world_file
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
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.10'

        ],

        output='screen'

    )


    return LaunchDescription([

        gazebo,

        robot_state_publisher,

        spawn_robot

    ])