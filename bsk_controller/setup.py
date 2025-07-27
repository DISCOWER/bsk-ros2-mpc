from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bsk_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['bsk_controller', 'bsk_controller.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('bsk_controller/launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elias Krantz',
    maintainer_email='eliaskra@kth.se',
    description='Basilisk ROS2 controller package',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'bsk_mpc = bsk_controller.bsk_mpc:main',
            'bsk_mpc_px4 = bsk_controller.bsk_mpc_px4:main',
        ],
    },
)