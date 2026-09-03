import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('agv_robot')
    config_file = os.path.join(pkg_share, 'config', 'localization_health.yaml')

    health_node = Node(
        package="agv_robot",
        executable="localization_health",
        name="localization_health",
        output="screen",
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        health_node
    ])
