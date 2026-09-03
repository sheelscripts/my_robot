import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # URDF / Xacro
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')
        ),
        (
            os.path.join('share', package_name, 'urdf', 'config'),
            glob('urdf/config/*.xacro')
        ),

        # Gazebo Harmonic worlds
        (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf') + glob('worlds/*.world')
        ),

        # Launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # Configuration files
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),

        # Maps
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*')
        ),

        # RViz configurations
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='rosuser',
    maintainer_email='user@robot.local',

    description='AGV Robot Simulation, SLAM and Nav2 Navigation',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'localization_health = agv_robot.localization:main',
        ],
    },
)