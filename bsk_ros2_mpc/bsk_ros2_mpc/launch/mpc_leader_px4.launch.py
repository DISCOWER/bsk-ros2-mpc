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
    name_others_arg = DeclareLaunchArgument(
        'name_others',
        default_value='crackle pop',
        description='Names of other spacecraft, separated by space'
    )
    period_arg = DeclareLaunchArgument(
        'period',
        default_value='20.0',
        description='Period (seconds) to stay at each waypoint'
    )

    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    name_others = LaunchConfiguration('name_others')
    period = LaunchConfiguration('period')

    ld = LaunchDescription()
    ld.add_action(namespace_arg)
    ld.add_action(type_arg)
    ld.add_action(name_others_arg)
    ld.add_action(period_arg)

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='bsk-mpc-px4',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'type': type},
            {'name_others': name_others}
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
            {'is_simulation': False}
        ],
    ))

    # Uncomment to publish static transforms for camera and inertial frame visualization
    # Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_world_to_inertial',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'inertial']
    # )
    # Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_world_to_camera',
    #     arguments=['2', '1.9', '2.3', '0.3010647', '0.3013046', '-0.6395013', '0.6400107', 'map', 'camera_link'] # camera 2
    # )

    return ld