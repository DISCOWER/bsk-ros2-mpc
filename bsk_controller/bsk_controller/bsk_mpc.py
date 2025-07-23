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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from bsk_msgs.msg import CmdForceBodyMsgPayload, CmdTorqueBodyMsgPayload, SCStatesMsgPayload, THRArrayCmdForceMsgPayload, HillRelStateMsgPayload
from .tools.path_tools import sample_other_path
from .tools.utils import MRP2quat

class BskMpc(Node):
    def __init__(self):
        super().__init__('bsk_mpc',
                        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # Get type
        self.type = self.declare_parameter('type', 'da').value

        # Get use_hill (true/false)
        self.use_hill = self.declare_parameter('use_hill', False).value

        # Get names of other spacecraft
        self.name_others = self.declare_parameter('name_others', '').value
        self.name_others = self.name_others.split() if self.name_others else []
        self.n_others = len(self.name_others)

        # Setup publishers and subscribers
        self.set_publishers_subscribers()

        # Check if use_sim_time is enabled
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info("Using simulation time, waiting for /clock...")
            self.wait_for_clock()

        timer_period_cmd = 0.1  # seconds
        self.timer_cmd = self.create_timer(timer_period_cmd, self.cmdloop_callback)

        # Create Spacecraft and controller objects
        if self.type == 'da':
            from bsk_controller.controllers.mpc_da import MpcDa
            self.mpc = MpcDa()
            self.control = np.zeros((self.mpc.nu, 1))

        elif self.type == 'wrench':
            from bsk_controller.controllers.mpc_wrench import MpcWrench
            self.mpc = MpcWrench()
            self.control = np.zeros((self.mpc.nu, 1))

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.setpoint_attitude = np.array([1.0, 0.0, 0.0, 0.0])

        # Initialize others' odometry and predicted path
        self.others = {
            name: {
                "odom": {
                    "timestamp": np.zeros(1),
                    "position": np.zeros(3),
                    "velocity": np.zeros(3),
                    # "acceleration": np.zeros(3),
                },
                "pred_path": {
                    "timestamps": [],
                    "positions": [],
                }
            }
            for name in self.name_others
        }

    def wait_for_clock(self):
        """Wait for the /clock topic to start publishing."""
        rate = self.create_rate(10)  # 10 Hz
        warned = False
        while rclpy.ok():
            topics = dict(self.get_topic_names_and_types())
            if '/clock' in topics:
                return
            if not warned:
                self.get_logger().info("Waiting for /clock topic...")
                warned = True
            rate.sleep()

    def vector2PoseMsg(self, frame_id, position, attitude):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.orientation.w = attitude[0]
        pose_msg.pose.orientation.x = attitude[1]
        pose_msg.pose.orientation.y = attitude[2]
        pose_msg.pose.orientation.z = attitude[3]
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        return pose_msg

    def set_publishers_subscribers(self):        
        # Subscribers
        self.state_sub = self.create_subscription(
            SCStatesMsgPayload, 
            "bsk/out/sc_states", 
            self.state_callback, 
            10
        )
        if self.use_hill:
            self.hill_state_sub = self.create_subscription(
                HillRelStateMsgPayload,
                "bsk/out/hill_rel_state",
                self.hill_state_callback,
                10
            )
        self.setpoint_pose_sub = self.create_subscription(
            PoseStamped,
            'bsk_mpc/setpoint_pose',
            self.get_setpoint_pose_callback,
            0
        )
    
        # Publishers
        self.predicted_path_pub = self.create_publisher(
            Path,
            'bsk_mpc/predicted_path',
            10)
        self.reference_pub = self.create_publisher(
            Marker,
            'bsk_mpc/reference',
            10)
        if self.type == 'da':
            self.publisher_thruster_array_cmd = self.create_publisher(
                THRArrayCmdForceMsgPayload, 
                "bsk/in/thr_array_cmd_force", 
                10
            )
        elif self.type == 'wrench':
            self.publisher_force_cmd = self.create_publisher(
                CmdForceBodyMsgPayload, 
                "bsk/in/cmd_force", 
                10
            )
            self.publisher_torque_cmd = self.create_publisher(
                CmdTorqueBodyMsgPayload,
                "bsk/in/cmd_torque",
                10
            )
        else:
            self.get_logger().error(f"Unknown type: {self.type}. Use 'direct_allocation' or 'wrench'.")
            return

    def state_callback(self, msg: SCStatesMsgPayload):
        # position and velocity in inertial frame
        # attitude in body to inertial frame
        # angular velocity in body frame
        if self.use_hill is False:
            self.vehicle_local_position = msg.r_bn_n
            self.vehicle_local_velocity = msg.v_bn_n
        sigma_bn = np.array(msg.sigma_bn)
        self.vehicle_attitude = MRP2quat(sigma_bn)
        self.vehicle_angular_velocity = msg.omega_bn_b
    
    def hill_state_callback(self, msg: HillRelStateMsgPayload):
        # position and velocity in Hill frame
        self.vehicle_local_position = msg.r_dc_h
        self.vehicle_local_velocity = msg.v_dc_h

    def others_odom_callback(self, msg: Odometry, namespace):
        odom = self.others[namespace]['odom']
        timestamp = self.get_clock().now().nanoseconds   #1e9 * msg.header.stamp.sec + msg.header.stamp.nanosec

        odom['position'][0] = msg.pose.pose.position.x
        odom['position'][1] = msg.pose.pose.position.y
        odom['position'][2] = msg.pose.pose.position.z

        odom['velocity'][0] = msg.twist.twist.linear.x
        odom['velocity'][1] = msg.twist.twist.linear.y
        odom['velocity'][2] = msg.twist.twist.linear.z

        odom['timestamp'] = timestamp

    def others_pred_callback(self, msg: Path, namespace):
        pred = self.others[namespace]['pred_path']

        # Reset stored prediction
        pred['timestamps'].clear()
        pred['positions'].clear()

        for pose_stamped in msg.poses:
            t = pose_stamped.header.stamp
            timestamp_ns = 1e9 * t.sec + t.nanosec

            pos = np.array([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            ])

            pred['timestamps'].append(timestamp_ns)
            pred['positions'].append(pos)

    def publish_reference(self, pub, reference):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "arrow"
        msg.id = 1
        msg.type = Marker.SPHERE
        msg.scale.x = 0.5
        msg.scale.y = 0.5
        msg.scale.z = 0.5
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        msg.pose.position.x = reference[0]
        msg.pose.position.y = reference[1]
        msg.pose.position.z = reference[2]
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        pub.publish(msg)

    def publish_thruster_cmd(self, u):
        Fthr = 1.5
        u = np.asarray(u).flatten()

        thr_array_msg = THRArrayCmdForceMsgPayload()
        thr_array_msg.stamp = self.get_clock().now().to_msg()

        # Generate actuator outputs dynamically
        thrust_command = []
        for t in u:
            thrust_command.extend([max(t, 0.0), max(-t, 0.0)])
        thrust_command = np.clip(np.array(thrust_command, dtype=np.float32), 0.0, 1.0)

        thr_array_msg.thrforce[:len(thrust_command)] = thrust_command * Fthr
        self.publisher_thruster_array_cmd.publish(thr_array_msg)

    def publish_wrench_cmd(self, u):
        force_msg = CmdForceBodyMsgPayload()
        force_msg.stamp = self.get_clock().now().to_msg()

        torque_msg = CmdTorqueBodyMsgPayload()
        torque_msg.stamp = self.get_clock().now().to_msg()

        force_msg.forcerequestbody = u[:3]
        torque_msg.torquerequestbody = u[3:6]

        self.publisher_force_cmd.publish(force_msg)
        self.publisher_torque_cmd.publish(torque_msg)

    def publish_predicted_path(self, x_pred, current_attitude):
        now = self.get_clock().now()
        predicted_path_msg = Path()
        predicted_path_msg.header.stamp = now.to_msg()
        predicted_path_msg.header.frame_id = 'map'

        for i, predicted_state in enumerate(x_pred):
            # Calculate future time offset
            future_time = now + rclpy.duration.Duration(seconds=i * self.mpc.dt)

            # Create PoseStamped
            pose_stamped = self.vector2PoseMsg('map', predicted_state[0:3], current_attitude)
            pose_stamped.header.stamp = future_time.to_msg()
            pose_stamped.header.frame_id = 'map'

            predicted_path_msg.poses.append(pose_stamped)

    def cmdloop_callback(self):
        x0 = np.array([self.vehicle_local_position[0],
                self.vehicle_local_position[1],
                self.vehicle_local_position[2],
                self.vehicle_local_velocity[0],
                self.vehicle_local_velocity[1],
                self.vehicle_local_velocity[2],
                self.vehicle_attitude[0],
                self.vehicle_attitude[1],
                self.vehicle_attitude[2],
                self.vehicle_attitude[3],
                self.vehicle_angular_velocity[0],
                self.vehicle_angular_velocity[1],
                self.vehicle_angular_velocity[2]]).reshape(13, 1)

        # Set state and references for each MPC
        if self.type == 'da' or self.type == 'wrench':
            x_ref = np.concatenate((self.setpoint_position,     # position
                                  np.zeros(3),                  # velocity
                                  self.setpoint_attitude,       # attitude
                                  np.zeros(3)), axis=0)         # angular velocity
            x_ref = np.repeat(x_ref.reshape((-1, 1)), self.mpc.Nx + 1, axis=1)
            
            # Get control input
            self.control, x_pred = self.mpc.get_input(x0, x_ref)
                                   
        elif self.type == 'avoidance_mpc':
            raise ValueError("Avoidance MPC is not implemented yet.")
            x_ref = np.concatenate((self.setpoint_position,       # position
                                  np.zeros(3),                  # velocity
                                  self.setpoint_attitude,       # attitude
                                  np.zeros(3)), axis=0)
            
            use_other_traj = True
            if use_other_traj:
                # Use predicted paths of other spacecraft
                x_others = [
                    sample_other_path(
                        t0=self.get_clock().now().nanoseconds,
                        dt=self.mpc.dt,
                        Nx=self.mpc.Nx + 1,
                        t_other=self.others[name]['pred_path']['timestamps'],
                        pos_other=self.others[name]['pred_path']['positions'],
                    )
                    for name in self.name_others if self.others[name]['pred_path']['timestamps']
                ]
            else:
                # Use current odometry of other spacecraft
                x_others = [
                    sample_other_path(
                        t0=self.get_clock().now().nanoseconds,
                        dt=self.mpc.dt,
                        Nx=self.mpc.Nx + 1,
                        t_other=[self.others[name]['odom']['timestamp']],
                        pos_other=[self.others[name]['odom']['position']],
                        vel_other=[self.others[name]['odom']['velocity']],
                    )
                    for name in self.name_others
                ]
            # Get control input
            self.control, x_pred = self.mpc.get_input(x0, x_ref, x_others=x_others)

        else:
            raise ValueError(f'Invalid type: {self.type}')


        self.publish_predicted_path(x_pred, self.setpoint_attitude)
        self.publish_reference(self.reference_pub, self.setpoint_position)

        if self.type == 'da' or self.type == 'avoidance_mpc':
            self.publish_thruster_cmd(self.control)
        elif self.type == 'wrench':
            self.publish_wrench_cmd(self.control)

    def add_set_pos_callback(self, request, response):
        self.setpoint_position[0] = request.pose.position.x
        self.setpoint_position[1] = request.pose.position.y
        self.setpoint_position[2] = request.pose.position.z
        self.setpoint_attitude[0] = request.pose.orientation.w
        self.setpoint_attitude[1] = request.pose.orientation.x
        self.setpoint_attitude[2] = request.pose.orientation.y
        self.setpoint_attitude[3] = request.pose.orientation.z
        return response

    def get_setpoint_pose_callback(self, msg):
        self.setpoint_position[0] = msg.pose.position.x
        self.setpoint_position[1] = msg.pose.position.y
        self.setpoint_position[2] = msg.pose.position.z
        self.setpoint_attitude[0] = msg.pose.orientation.w
        self.setpoint_attitude[1] = msg.pose.orientation.x
        self.setpoint_attitude[2] = msg.pose.orientation.y
        self.setpoint_attitude[3] = msg.pose.orientation.z

        # normalize setpoint attitude
        norm = np.linalg.norm(self.setpoint_attitude)
        if norm > 0:
            self.setpoint_attitude /= norm

        self.get_logger().info(f"Setpoint position: {self.setpoint_position}, attitude: {self.setpoint_attitude}")
        self.get_logger().info(f"Position: {self.vehicle_local_position}, Attitude: {self.vehicle_attitude}")


def main(args=None):
    rclpy.init(args=args)
    bsk_mpc = BskMpc()
    rclpy.spin(bsk_mpc)
    bsk_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
