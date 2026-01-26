#!/usr/bin/env python
'''  Launch 2 follower MPC's and setpoint publishers '''
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
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='True',
        description='Use Hill frame for MPC'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_hill = LaunchConfiguration('use_hill')

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_hill_arg)

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='followerSc_1',
        executable='bsk-mpc',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'type': 'follower_wrench'},
            {'use_hill': use_hill},
            {'name_leader': 'leaderSc'}
        ]
    ))
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='followerSc_2',
        executable='bsk-mpc',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'type': 'follower_wrench'},
            {'use_hill': use_hill},
            {'name_leader': 'leaderSc'}
        ]
    ))

    # Launch follower publisher
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='followerSc_1',
        executable='follower-publisher',
        name='follower_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'position': [-1.0, 0.3, 0.0]},
            {'is_sim': True}
        ]
    ))
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace='followerSc_2',
        executable='follower-publisher',
        name='follower_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'position': [-1.0, -0.3, 0.0]},
            {'is_sim': True}
        ]
    ))

    return ld