import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_dir = get_package_share_directory('agv_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'agv.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.sdf')
    
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toprettyxml(indent='  ')

    return LaunchDescription([
        # Gazebo Sim
        ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen'),
        
        # Robot State Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]),
             
        # Spawn Robot
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'agv']),
             
        # ROS-GZ Bridge (cmd_vel, odom, tf, lidar, joint_states, clock)
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                 '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                 '/front_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                 '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
             ],
             remappings=[
                 ('/front_lidar/scan', '/scan')
             ])
    ])
