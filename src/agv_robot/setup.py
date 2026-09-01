import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install URDF and xacro files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf/config'), glob('urdf/config/*.xacro')),
        # Install Gazebo worlds
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf') + glob('worlds/*.world')),
        # Install Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install Configs & Maps
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='user@robot.local',
    description='AGV Robot Simulation, SLAM and Nav2 Navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
