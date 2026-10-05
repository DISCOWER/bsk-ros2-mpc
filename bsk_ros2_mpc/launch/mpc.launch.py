#!/usr/bin/env python
''' Launch the MPC node '''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os
import tempfile

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time from /clock topic'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bskSat',
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
    name_leader_arg = DeclareLaunchArgument(
        'name_leader',
        default_value='',
        description='Namespace of the leader spacecraft'
    )
    name_others_arg = DeclareLaunchArgument(
        'name_others',
        default_value='',
        description='Namespaces of other spacecraft, separated by spaces'
    )
    rviz_mode_arg = DeclareLaunchArgument(
        'rviz_mode',
        default_value='setpoint',
        description='RViz mode: setpoint, viz or off'
    )
    skip_build_arg = DeclareLaunchArgument(
        'skip_build',
        default_value='False',
        description='Skip building acados solver (set to True to use existing compiled code)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    name_leader = LaunchConfiguration('name_leader')
    name_others = LaunchConfiguration('name_others')
    rviz_mode = LaunchConfiguration('rviz_mode')
    skip_build = LaunchConfiguration('skip_build')

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(type_arg)
    ld.add_action(use_hill_arg)
    ld.add_action(name_leader_arg)
    ld.add_action(name_others_arg)
    ld.add_action(rviz_mode_arg)
    ld.add_action(skip_build_arg)

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='bsk-mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'type': type},
            {'use_hill': use_hill},
            {'name_leader': name_leader},
            {'name_others': name_others},
            {'rviz_mode': ParameterValue(rviz_mode, value_type=str)},
            {'skip_build': skip_build}

        ]
    ))

    # Launch RViz pose marker
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='rviz_pose_marker',
        name='rviz_pose_marker',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' == 'setpoint'"]))
    ))

    # Launch visualizer
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='visualizer',
        name='visualizer',
        parameters=[
            {'use_hill': use_hill},
            {'name_others': name_others}
        ],
        condition=IfCondition(PythonExpression(["'", rviz_mode, "' != 'off'"]))
    ))

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld

def patch_rviz_config(original_config_path, namespace):
    """
    Patch the RViz configuration file to replace the namespace placeholder with the actual namespace.
    """
    with open(original_config_path, 'r') as f:
        content = f.read()

    # Replace placeholder with actual namespace
    content = content.replace('__NS__', f'/{namespace}' if namespace else '')

    # Write to temporary file
    tmp_rviz_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    tmp_rviz_config.write(content.encode('utf-8'))
    tmp_rviz_config.close()

    return tmp_rviz_config.name


def launch_setup(context, *args, **kwargs):
    """
    Function to set up the launch context and patch the RViz configuration.
    """
    namespace = LaunchConfiguration('namespace').perform(context)
    rviz_config_path = os.path.join(get_package_share_directory('bsk-ros2-mpc'), 'config', 'config.rviz')
    patched_config = patch_rviz_config(rviz_config_path, namespace)

    return [
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', patched_config],
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rviz_mode'), "' != 'off'"]))
        )
    ]