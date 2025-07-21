#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


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
    setpoint_from_rviz_arg = DeclareLaunchArgument(
        'setpoint_from_rviz',
        default_value='False',
        description='Publish setpoint pose via rviz'
    )
    name_others_arg = DeclareLaunchArgument(
        'name_others',
        default_value='',
        description='Namespaces of other spacecraft, separated by space'
    )
    namespace = LaunchConfiguration('namespace')
    type = LaunchConfiguration('type')
    use_hill = LaunchConfiguration('use_hill')
    setpoint_from_rviz = LaunchConfiguration('setpoint_from_rviz')
    name_others = LaunchConfiguration('name_others')

    return LaunchDescription([
        namespace_arg,
        type_arg,
        use_hill_arg,
        setpoint_from_rviz_arg,
        name_others_arg,

        # Launch MPC
        Node(
            package='bsk_controller',
            namespace=namespace,
            executable='bsk_mpc',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': type},
                {'use_hill': use_hill},
                {'setpoint_from_rviz': setpoint_from_rviz},
                {'name_others': name_others}
            ]
        ),
        # Launch RViz marker input (only if using RViz to control)
        Node(
            package='bsk_controller',
            namespace=namespace,
            executable='rviz_pos_marker',
            name='rviz_pos_marker',
            output='screen',
            emulate_tty=True,
            parameters=[{'namespace': namespace}],
            condition=IfCondition(setpoint_from_rviz)
        ),
        # Always launch main robot visualiser
        # Node(
        #     package='bsk_offboard',
        #     namespace=namespace,
        #     executable='visualizer',
        #     name='visualizer',
        #     parameters=[{'namespace': namespace}],
        #     output='screen'
        # ),
        # Launch visualisers for others + RViz
        # OpaqueFunction(function=launch_visualizers_and_rviz),
    ])


def patch_rviz_config(template_path, ns):
    with open(template_path, 'r') as f:
        content = f.read()

    content = content.replace('__NS__', f'/{ns}' if ns else '')

    tmp_config = tempfile.NamedTemporaryFile(delete=False, suffix='.rviz')
    tmp_config.write(content.encode('utf-8'))
    tmp_config.close()

    return tmp_config.name


def launch_visualizers_and_rviz(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    setpoint_from_rviz = LaunchConfiguration('setpoint_from_rviz').perform(context)

    # Choose config depending on RViz control
    base_config_name = 'config.rviz' if setpoint_from_rviz == 'true' else 'config_nomarker.rviz'

    rviz_config_path = os.path.join(
        get_package_share_directory('bsk_controller'),
        'rviz',
        base_config_name
    )
    patched_config = patch_rviz_config(rviz_config_path, namespace)

    return [
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', patched_config],
            output='screen'
        )
    ]