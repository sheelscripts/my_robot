import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_agv_robot = get_package_share_directory('agv_robot')
    
    # Launch scan_2d_merger as a composable node
    scan_merger_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='scan_merger_container',
        namespace='',
        composable_node_descriptions=[
            ComposableNode(
                package='scan_2d_merger',
                plugin='util::LaserScanMerger',
                name='scan_2d_merger_node',
                parameters=[os.path.join(pkg_agv_robot, 'config', 'scan_merger.yaml')]
            )
        ],
        output='screen'
    )
    
    pkg_agv_robot = get_package_share_directory('agv_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch Nav2
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'map': os.path.join(pkg_agv_robot, 'maps', 'warehouse_map.yaml'),
            'params_file': os.path.join(pkg_agv_robot, 'config', 'nav2_params.yaml')
        }.items()
    )
    
    # Launch RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'namespace': '', 'use_namespace': 'False'}.items()
    )
    
    return LaunchDescription([
        scan_merger_container,
        nav2_bringup_cmd,
        rviz_cmd
    ])
