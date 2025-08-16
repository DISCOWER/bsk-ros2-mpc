#!/usr/bin/env python
''' Launch the MPC node '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bskSat',
        description='Namespace for all nodes'
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='da',
        description='Type of the controller (da, wrench, ...)'
    )
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='False',
        description='Use Hill frame for MPC'
    )
    name_leader_arg = DeclareLaunchArgument(
        'name_leader',
        default_value='',
        description='Namespace of the leader spacecraft'
    )
    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    name_leader = LaunchConfiguration('name_leader')

    return LaunchDescription([
        namespace_arg,
        type_arg,
        use_hill_arg,
        name_leader_arg,

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
                {'use_hill': use_hill},
                {'name_leader': name_leader}
            ]
        ),
    ])