#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from geometry_msgs.msg import Point
from interactive_markers import InteractiveMarkerServer, MenuHandler
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from bsk_mpc_msgs.srv import SetPose

def makeBox(msg):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.33
    marker.scale.y = msg.scale * 0.33
    marker.scale.z = msg.scale * 0.33
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    if norm == 0.0:
        raise ValueError('Cannot normalize a zero quaternion')
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def make6DofMarker(server, menu_handler, process_feedback, fixed, interaction_mode, position, show_6dof=False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.pose.position = position
    int_marker.scale = 0.36

    int_marker.name = 'simple_6dof'

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = InteractiveMarkerControl.MENU

    if fixed:
        int_marker.name += '_fixed'

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: 'MOVE_3D',
            InteractiveMarkerControl.ROTATE_3D: 'ROTATE_3D',
            InteractiveMarkerControl.MOVE_ROTATE_3D: 'MOVE_ROTATE_3D'
        }
        int_marker.name += '_' + control_modes_dict[interaction_mode]

    if show_6dof:
        for axis, name in [(1.0, 'move_x'), (2.0, 'move_y'), (3.0, 'move_z')]:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = float(axis == 1.0)
            control.orientation.y = float(axis == 2.0)
            control.orientation.z = float(axis == 3.0)
            normalizeQuaternion(control.orientation)
            control.name = name
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

        # Rotation controls
        for axis, name in [(1.0, 'rotate_x'), (2.0, 'rotate_y'), (3.0, 'rotate_z')]:
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = float(axis == 1.0)
            control.orientation.y = float(axis == 2.0)
            control.orientation.z = float(axis == 3.0)
            normalizeQuaternion(control.orientation)
            control.name = name
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

    server.insert(int_marker, feedback_callback=process_feedback)
    menu_handler.apply(server, int_marker.name)

class RvizPoseMarker(Node):
    def __init__(self):
        super().__init__('rviz_target_pose_marker')

        self.set_pose_client = self.create_client(SetPose, 'set_pose')
        self.menu_handler = MenuHandler()
        self.server = InteractiveMarkerServer(self, 'rviz_target_pose_marker')

        self.menu_handler.insert('Command Pose', callback=self.command_pose_callback)
        self.menu_handler.insert('Reset')

        position = Point(x=1.0, y=1.0, z=0.0)
        make6DofMarker(self.server, self.menu_handler, self.process_feedback, True, InteractiveMarkerControl.NONE, position, True)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        log_prefix = (
            f"Feedback from marker '{feedback.marker_name}' / control '{feedback.control_name}'"
        )

        log_mouse = ''
        if feedback.mouse_point_valid:
            log_mouse = (
                f'{feedback.mouse_point.x}, {feedback.mouse_point.y}, '
                f'{feedback.mouse_point.z} in frame {feedback.header.frame_id}'
            )

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info(f'{log_prefix}: button click at {log_mouse}')
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.get_logger().debug(f'{log_prefix}: mouse down at {log_mouse}')
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().debug(f'{log_prefix}: mouse up at {log_mouse}')

    def command_pose_callback(self, feedback):
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warn("Service 'set_pose' not available, is the MPC running?")
            return

        request = SetPose.Request()
        request.pose = feedback.pose
        future = self.set_pose_client.call_async(request)
        future.add_done_callback(self.command_pose_response_callback)

        position = feedback.pose.position
        orientation = feedback.pose.orientation
        self.get_logger().info(
            f'Commanded pose: position=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), '
            f'attitude=({orientation.w:.3f}, {orientation.x:.3f}, '
            f'{orientation.y:.3f}, {orientation.z:.3f})'
        )

    def command_pose_response_callback(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to command pose: {error}')
            return

        if not response.result:
            self.get_logger().warn('MPC rejected the commanded pose')

    def alignMarker(self, feedback):
        pose = feedback.pose

        pose.position.x = round(pose.position.x - 0.5) + 0.5
        pose.position.y = round(pose.position.y - 0.5) + 0.5

        self.get_logger().info(
            f'{feedback.marker_name}: aligning position = {feedback.pose.position.x}, '
            f'{feedback.pose.position.y}, {feedback.pose.position.z} to '
            f'{pose.position.x}, {pose.position.y}, {pose.position.z}'
        )

        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

def main(args=None):
    rclpy.init(args=args)
    node = RvizPoseMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
