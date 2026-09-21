from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_simulation'),
                'launch',
                'navigation_icp.launch.py'
            ])
        )
    )


    ground_truth = Node(
        package='localization_evaluation',
        executable='ground_truth_node',
        name='ground_truth_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )


    benchmark_runner = Node(
        package='localization_evaluation',
        executable='localization_benchmark_runner',
        name='localization_benchmark_runner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )


    return LaunchDescription([
        navigation,
        ground_truth,
        benchmark_runner,
    ])
