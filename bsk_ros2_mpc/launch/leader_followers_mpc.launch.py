#!/usr/bin/env python
'''Launch a leader, follower MPCs, setpoint publishers, and shared visualization.'''

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    leader_arg = DeclareLaunchArgument(
        'leader',
        default_value='leaderSc',
        description='Namespace of the leader spacecraft',
    )
    followers_arg = DeclareLaunchArgument(
        'followers',
        default_value='followerSc_1 followerSc_2',
        description='Follower namespaces separated by spaces',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time from /clock topic',
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='wrench',
        description='Leader controller type: da or wrench',
    )
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='True',
        description='Use Hill frame for MPC',
    )
    period_arg = DeclareLaunchArgument(
        'period',
        default_value='20.0',
        description='Period in seconds to stay at each leader waypoint',
    )
    rviz_mode_arg = DeclareLaunchArgument(
        'rviz_mode',
        default_value='viz',
        description='RViz mode: viz or off',
    )

    ld = LaunchDescription()
    ld.add_action(leader_arg)
    ld.add_action(followers_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(type_arg)
    ld.add_action(use_hill_arg)
    ld.add_action(period_arg)
    ld.add_action(rviz_mode_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def launch_setup(context, *args, **kwargs):
    leader = LaunchConfiguration('leader').perform(context)
    agents = LaunchConfiguration('followers').perform(context).split()
    if len(set(agents)) != len(agents):
        raise RuntimeError("The 'followers' launch argument must not contain duplicates")
    if leader in agents:
        raise RuntimeError("The leader namespace must not also be listed in 'followers'")

    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    period = LaunchConfiguration('period')
    rviz_mode = LaunchConfiguration('rviz_mode')
    actions = []

    # Launch the leader MPC and its waypoint publisher.
    actions.append(Node(
        package='bsk-ros2-mpc',
        namespace=leader,
        executable='bsk-mpc',
        name='bsk_mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'type': controller_type},
            {'use_hill': use_hill},
            {'rviz_mode': 'off'},
        ],
    ))
    actions.append(Node(
        package='bsk-ros2-mpc',
        namespace=leader,
        executable='waypoint-publisher',
        name='waypoint_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'period': period},
            {'is_sim': True},
        ],
    ))

    # Launch one follower MPC and setpoint publisher per requested namespace.
    for index, namespace in enumerate(agents):
        actions.append(Node(
            package='bsk-ros2-mpc',
            namespace=namespace,
            executable='bsk-mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'type': 'follower_wrench'},
                {'use_hill': use_hill},
                {'name_leader': leader},
                {'rviz_mode': 'off'},
            ],
        ))
        offset = 0.3 if index % 2 == 0 else -0.3
        actions.append(Node(
            package='bsk-ros2-mpc',
            namespace=namespace,
            executable='follower-publisher',
            name='follower_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'position': [-1.0, offset, 0.0]},
                {'is_sim': True},
            ],
        ))

    visualizer_agents = [leader] + agents
    rviz_config_path = os.path.join(
        get_package_share_directory('bsk-ros2-mpc'),
        'config',
        'config.rviz',
    )
    patched_config = patch_rviz_config(rviz_config_path, visualizer_agents)

    # Visualize the complete leader-follower formation in one RViz instance.
    actions.append(Node(
        package='bsk-ros2-mpc',
        namespace='',
        executable='visualizer',
        name='visualizer',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_hill': use_hill},
            {'agents': ' '.join(visualizer_agents)},
        ],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'off'"])),
    ))
    actions.append(Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', patched_config],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'off'"])),
    ))
    return actions


def patch_rviz_config(original_config_path, agents):
    """Create an RViz config for the leader-follower formation."""
    with open(original_config_path, 'r') as config_file:
        content = config_file.read()

    displays_marker = '  Displays:\n'
    enabled_marker = '  Enabled: true\n  Global Options:'
    displays_start = content.index(displays_marker) + len(displays_marker)
    enabled_start = content.index(enabled_marker, displays_start)
    display_blocks = re.split(r'(?=    - )', content[displays_start:enabled_start])

    patched_blocks = []
    for block in display_blocks:
        if not block.strip() or 'Class: rviz_default_plugins/InteractiveMarkers' in block:
            continue
        if '__AGENT__' in block:
            for agent in agents:
                patched_blocks.append(block.replace('__AGENT__', agent))
        else:
            patched_blocks.append(block)

    patched_content = (
        content[:displays_start]
        + ''.join(patched_blocks)
        + content[enabled_start:]
    ).replace('__MARKER_NS__', '')

    temporary_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    temporary_config.write(patched_content.encode('utf-8'))
    temporary_config.close()
    return temporary_config.name
