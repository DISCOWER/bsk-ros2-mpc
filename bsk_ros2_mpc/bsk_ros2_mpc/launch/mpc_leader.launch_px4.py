#!/usr/bin/env python
''' Launch the leader MPC and waypoint publisher with PX4 messages '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='snap',
        description='Namespace for all nodes'
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='wrench',
        description='Type of the controller (da, wrench, ...)'
    )
    period_arg = DeclareLaunchArgument(
        'period',
        default_value='20.0',
        description='Period (seconds) to stay at each waypoint'
    )

    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    period = LaunchConfiguration('period')

    return LaunchDescription([
        namespace_arg,
        type_arg,
        period_arg,

        # Launch MPC
        Node(
            package='bsk-ros2-mpc',
            namespace=namespace,
            executable='bsk_mpc_px4',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': type}
            ]
        ),
        Node(
            package='bsk-ros2-mpc',
            namespace=namespace,
            executable='waypoint_publisher',
            name='waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'period': period},
                {'is_simulation': False}
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_inertial',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'inertial']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_camera',
            arguments=['2', '1.9', '2.3', '0.3010647', '0.3013046', '-0.6395013', '0.6400107', 'map', 'camera_link'] # camera 2
        ),
    ])