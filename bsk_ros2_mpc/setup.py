from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bsk-ros2-mpc'
package_dir = 'bsk_ros2_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_dir, 'bsk_ros2_mpc.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(package_dir +'/launch/*.launch.py')),
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
            'bsk-mpc = bsk_ros2_mpc.bsk_mpc:main',
            'bsk-mpc-px4 = bsk_ros2_mpc.bsk_mpc_px4:main',
            'follower-publisher = bsk_ros2_mpc.bsk_planner.follower_publisher:main',
            'waypoint-publisher = bsk_ros2_mpc.bsk_planner.waypoint_publisher:main',
        ],
    },
)