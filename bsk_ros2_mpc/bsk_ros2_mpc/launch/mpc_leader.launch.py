#!/usr/bin/env python
''' Launch Leader's MPC and setpoint publisher '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time from /clock topic'
    )
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    period = LaunchConfiguration('period')

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(type_arg)
    ld.add_action(use_hill_arg)
    ld.add_action(period_arg)

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='bsk-mpc',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'type': type},
            {'use_hill': use_hill}
        ]
    ))

    # Launch waypoint publisher
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='waypoint-publisher',
        name='waypoint_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'period': period},
            {'is_sim': True}
        ],
    ))

    return ld