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

    # TEMPORARY:
    # 현재 icp_localization_node는 map_to_odom_을 내부적으로만 갱신하고
    # 실제 TF broadcaster로 map -> odom을 아직 발행하지 않는다.
    # 따라서 RViz / TF tree 검증을 위해 identity static TF를 유지한다.
    # 이후 ICP node가 map -> odom을 직접 broadcast하게 되면 반드시 제거한다.
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='temporary_map_to_odom_static_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ]
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
        static_map_to_odom,
        icp_localization,
    ])
