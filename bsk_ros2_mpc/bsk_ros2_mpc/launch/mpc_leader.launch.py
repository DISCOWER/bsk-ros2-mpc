#!/usr/bin/env python
''' Launch Leader's MPC and setpoint publisher '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='leaderSc',
        description='Namespace for all nodes'
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='wrench',
        description='Type of the controller (da, wrench, ...)'
    )
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='True',
        description='Use Hill frame for MPC'
    )
    period_arg = DeclareLaunchArgument(
        'period',
        default_value='20.0',
        description='Period (seconds) to stay at each waypoint'
    )

    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    period = LaunchConfiguration('period')

    return LaunchDescription([
        namespace_arg,
        type_arg,
        use_hill_arg,
        period_arg,

        # Launch MPC
        Node(
            package='bsk-ros2-mpc',
            namespace=namespace,
            executable='bsk_mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': type},
                {'use_hill': use_hill}
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
                {'is_sim': True}
            ],
        ),
    ])