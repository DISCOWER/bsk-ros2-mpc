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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch MPC
        Node(
            package='bsk_controller',
            namespace='crackle',
            executable='bsk_mpc_px4',
            name='bsk_mpc_px4',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': 'follower_wrench'},
                {'name_leader': 'snap'}
            ]
        ),
        Node(
            package='bsk_controller',
            namespace='crackle',
            executable='follower_waypoint_publisher',
            name='follower_waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'position': [-1.0, 0.3, 0.0]},
                {'is_simulation': False}
            ]
        ),
        Node(
            package='bsk_controller',
            namespace='pop',
            executable='bsk_mpc_px4',
            name='bsk_mpc',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'type': 'follower_wrench'},
                {'name_leader': 'snap'}
            ]
        ),
        Node(
            package='bsk_controller',
            namespace='pop',
            executable='follower_waypoint_publisher',
            name='follower_waypoint_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'position': [-1.0, -0.3, 0.0]},
                {'is_simulation': False}
            ]
        ),
    ])