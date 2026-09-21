from ament_index_python.packages import get_package_share_directory

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'ekf_localization'
    )

    params_file = os.path.join(
        package_share,
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ekf_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': True
                }
            ],
        ),
    ])
