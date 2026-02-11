#!/usr/bin/env python
''' Launch the MPC node '''
__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
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
        default_value='da',
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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Launch RViz visualizer'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    name_leader = LaunchConfiguration('name_leader')
    use_rviz = LaunchConfiguration('use_rviz')

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(type_arg)
    ld.add_action(use_hill_arg)
    ld.add_action(name_leader_arg)
    ld.add_action(use_rviz_arg)

    # Launch MPC
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='bsk-mpc',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'use_rviz': use_rviz},
            {'type': type},
            {'use_hill': use_hill},
            {'name_leader': name_leader}
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
        condition=IfCondition(use_rviz)
    ))

    # Launch visualizer
    ld.add_action(Node(
        package='bsk-ros2-mpc',
        namespace=namespace,
        executable='visualizer',
        name='visualizer',
        parameters=[
            {'use_hill': use_hill}
        ],
        condition=IfCondition(use_rviz)
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
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ]