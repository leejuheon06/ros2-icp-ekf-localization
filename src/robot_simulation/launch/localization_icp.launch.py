from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

from lifecycle_msgs.msg import Transition
from launch.actions import EmitEvent


def generate_launch_description():
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    map_yaml = LaunchConfiguration('map')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_simulation'),
                'launch',
                'simulation.launch.py'
            ])
        ),
        launch_arguments={
            'world': world,
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw,
        }.items()
    )

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True,
        }]
    )

    configure_map_server = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(map_server),
                                transition_id=Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ]
                )
            ]
        )
    )

    activate_map_server = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(map_server),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    icp_localization = Node(
        package='icp_localization',
        executable='icp_localization_node',
        name='icp_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='localization_world.sdf'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='-3.8'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.10'
        ),
        DeclareLaunchArgument(
            'yaw',
            default_value='1.5708'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'ros2_icp_ekf_localization',
                'maps',
                'localization_map.yaml'
            ])
        ),

        simulation,
        map_server,
        configure_map_server,
        activate_map_server,
        icp_localization,
    ])
