#!/usr/bin/env python
'''Launch one MPC and visualizer per agent with one shared RViz window.'''

import os, re, tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch-time configuration.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time from /clock topic',
    )
    agents_arg = DeclareLaunchArgument(
        'agents',
        description='Spacecraft namespaces separated by spaces',
    )
    type_arg = DeclareLaunchArgument(
        'type',
        default_value='wrench',
        description='Type of the controller: da, wrench, or follower_wrench',
    )
    use_hill_arg = DeclareLaunchArgument(
        'use_hill',
        default_value='True',
        description='Use Hill frame for MPC',
    )
    rviz_mode_arg = DeclareLaunchArgument(
        'rviz_mode',
        default_value='setpoint',
        description='RViz mode: setpoint, viz or off',
    )
    skip_build_arg = DeclareLaunchArgument(
        'skip_build',
        default_value='False',
        description='Skip building acados solver code',
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(agents_arg)
    ld.add_action(type_arg)
    ld.add_action(use_hill_arg)
    ld.add_action(rviz_mode_arg)
    ld.add_action(skip_build_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld


def patch_rviz_config(original_config_path, agents):
    # Expand the RViz template once for every agent.
    """Create one RViz config containing displays for every agent."""
    with open(original_config_path, 'r') as config_file:
        content = config_file.read()

    displays_marker = '  Displays:\n'
    enabled_marker = '  Enabled: true\n  Global Options:'
    displays_start = content.index(displays_marker) + len(displays_marker)
    enabled_start = content.index(enabled_marker, displays_start)
    display_blocks = re.split(r'(?=    - )', content[displays_start:enabled_start])

    patched_blocks = []
    for block in display_blocks:
        if not block.strip():
            continue
        if '__AGENT__' in block:
            for agent in agents:
                patched_blocks.append(block.replace('__AGENT__', agent))
        elif block.strip():
            patched_blocks.append(block)

    patched_content = (
        content[:displays_start]
        + ''.join(patched_blocks)
        + content[enabled_start:]
    )
    patched_content = patched_content.replace('__MARKER_NS__', '')
    temporary_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    temporary_config.write(patched_content.encode('utf-8'))
    temporary_config.close()
    return temporary_config.name


def launch_setup(context, *args, **kwargs):
    agents = LaunchConfiguration('agents').perform(context).split()
    if not agents:
        raise RuntimeError("The 'agents' launch argument must contain at least one agent")
    if len(set(agents)) != len(agents):
        raise RuntimeError("The 'agents' launch argument must not contain duplicates")

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_hill = LaunchConfiguration('use_hill')
    type = LaunchConfiguration('type')
    rviz_mode = LaunchConfiguration('rviz_mode')
    skip_build = LaunchConfiguration('skip_build')

    # Create one MPC node and its collision-avoidance inputs per agent.
    mpc_nodes = []
    for agent in agents:
        other_agents = ' '.join(other for other in agents if other != agent)
        mpc_nodes.append(Node(
            package='bsk-ros2-mpc',
            namespace=agent,
            executable='bsk-mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'type': type},
                {'use_hill': use_hill},
                {'name_others': other_agents},
                {'rviz_mode': ParameterValue(rviz_mode, value_type=str)},
                {'skip_build': skip_build if agent == agents[0] else True},
            ],
        ))

    # Use one shared visualizer and RViz instance for all agents.
    visualizer_node = Node(
        package='bsk-ros2-mpc',
        namespace='',
        executable='visualizer',
        name='visualizer',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_hill': use_hill},
            {'agents': ' '.join(agents)},
        ],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'off'"])),
    )

    marker_node = Node(
        package='bsk-ros2-mpc',
        namespace='',
        executable='rviz_pose_marker',
        name='rviz_pose_marker',
        output='screen',
        emulate_tty=True,
        parameters=[{'agents': ' '.join(agents)}],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' == 'setpoint'"])),
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('bsk-ros2-mpc'),
        'config',
        'config.rviz',
    )
    patched_config = patch_rviz_config(rviz_config_path, agents)
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', patched_config],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'off'"])),
    )

    # Start the remaining nodes after the first solver is ready.
    pending_nodes = mpc_nodes[1:] + [visualizer_node, marker_node, rviz_node]
    ready = {'launched': False}

    def launch_pending_nodes(event):
        event_text = event.text
        if isinstance(event_text, bytes):
            event_text = event_text.decode(errors='replace')
        if ready['launched'] or 'MPC controller ready' not in event_text:
            return []
        ready['launched'] = True
        return pending_nodes

    return [
        RegisterEventHandler(OnProcessIO(
            target_action=mpc_nodes[0],
            on_stdout=launch_pending_nodes,
            on_stderr=launch_pending_nodes,
        )),
        mpc_nodes[0],
    ]