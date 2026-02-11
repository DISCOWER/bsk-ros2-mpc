#!/usr/bin/env python
''' Launch 2 follower MPC's and setpoint publishers with PX4 messages '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='crackle',
        executable='bsk_mpc_px4',
        name='bsk_mpc_px4',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'type': 'follower_wrench'},
            {'name_leader': 'snap'}
        ]
    ))

    # Launch follower waypoint publisher
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='crackle',
        executable='follower_waypoint_publisher',
        name='follower_waypoint_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'position': [-1.0, 0.3, 0.0]},
            {'is_simulation': False}
        ]
    ))

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='pop',
        executable='bsk_mpc_px4',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'type': 'follower_wrench'},
            {'name_leader': 'snap'}
        ]
    ))

    # Launch follower waypoint publisher
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='pop',
        executable='follower_waypoint_publisher',
        name='follower_waypoint_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'position': [-1.0, -0.3, 0.0]},
            {'is_simulation': False}
        ]
    ))

    return ld