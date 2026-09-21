from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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


    ekf_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ekf_localization'),
                'launch',
                'ekf_localization.launch.py'
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


    # -------------------------------------------------
    # Staggered startup
    # -------------------------------------------------
    #
    # 0 s  : Gazebo + ICP + Nav2
    # 10 s : EKF
    # 12 s : Ground Truth
    # 15 s : Benchmark Runner
    #
    # The benchmark runner still checks that bt_navigator is
    # ACTIVE before sending Point 1.
    #
    # TimerAction is used here only to reduce launch-time
    # contention between Gazebo, Nav2, ICP, EKF and evaluation.
    # -------------------------------------------------

    delayed_ekf = TimerAction(
        period=10.0,
        actions=[
            ekf_localization,
        ]
    )

    delayed_ground_truth = TimerAction(
        period=12.0,
        actions=[
            ground_truth,
        ]
    )

    delayed_benchmark_runner = TimerAction(
        period=15.0,
        actions=[
            benchmark_runner,
        ]
    )


    return LaunchDescription([
        navigation,
        delayed_ekf,
        delayed_ground_truth,
        delayed_benchmark_runner,
    ])
