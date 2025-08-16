#!/usr/bin/env python
'''  Launch 2 follower MPC's and setpoint publishers '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='True',
        description='Use Hill frame for MPC'
    )

    use_hill = LaunchConfiguration('use_hill')

    return LaunchDescription([
        use_hill_arg,

        Node(
            package='bsk-ros2-mpc',
            namespace='followerSc_1',
            executable='bsk_mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': 'follower_wrench'},
                {'use_hill': use_hill},
                {'name_leader': 'leaderSc'}
            ]
        ),
        Node(
            package='bsk-ros2-mpc',
            namespace='followerSc_2',
            executable='bsk_mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': 'follower_wrench'},
                {'use_hill': use_hill},
                {'name_leader': 'leaderSc'}
            ]
        ),
        Node(
            package='bsk-ros2-mpc',
            namespace='followerSc_1',
            executable='follower_waypoint_publisher',
            name='follower_waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'position': [-1.0, 0.3, 0.0]},
                {'is_sim': True}
            ]
        ),
        Node(
            package='bsk-ros2-mpc',
            namespace='followerSc_2',
            executable='follower_waypoint_publisher',
            name='follower_waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'position': [-1.0, -0.3, 0.0]},
                {'is_sim': True}
            ]
        ),
    ])