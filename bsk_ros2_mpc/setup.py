from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bsk-ros2-mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elias Krantz',
    maintainer_email='eliaskra@kth.se',
    description='Basilisk ROS2 controller package',
    license='BSD-3',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'bsk-mpc = bsk_ros2_mpc.nodes.bsk_mpc_node:main',
            'bsk-mpc-px4 = bsk_ros2_mpc.nodes.bsk_mpc_px4_node:main',
            'follower-publisher = bsk_ros2_mpc.planners.follower_publisher:main',
            'waypoint-publisher = bsk_ros2_mpc.planners.waypoint_publisher:main',
            'rviz_pose_marker = bsk_ros2_mpc.nodes.rviz_pose_marker_node:main',
            'visualizer = bsk_ros2_mpc.nodes.visualizer_node:main',
        ],
    },
)