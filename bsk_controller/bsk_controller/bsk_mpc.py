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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from bsk_msgs.msg import CmdForceBodyMsgPayload, CmdTorqueBodyMsgPayload, SCStatesMsgPayload, THRArrayCmdForceMsgPayload, HillRelStateMsgPayload, AttGuidMsgPayload
from .tools.utils import MRP2quat, sample_other_path

class BskMpc(Node):
    def __init__(self):
        super().__init__('bsk_mpc',
                        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # Get type
        self.type = self.declare_parameter('type', 'da').value

        # Get use_hill (true/false)
        self.use_hill = self.declare_parameter('use_hill', False).value

        # Get name of leader spacecraft
        self.name_leader = self.declare_parameter('name_leader', '').value

        # Setup publishers and subscribers
        self.set_publishers_subscribers()

        # Check if use_sim_time is enabled
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info("Using simulation time, waiting for /clock...")
            self.wait_for_clock()

        timer_period_cmd = 0.2  # seconds
        self.timer_cmd = self.create_timer(timer_period_cmd, self.cmdloop_callback)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.setpoint_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.setpoint_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.received_first_setpoint = False

        # Create Spacecraft and controller objects
        if self.type == 'da':
            from bsk_controller.controllers.mpc_da import MpcDa
            self.mpc = MpcDa()
            self.control = np.zeros((self.mpc.nu, 1))
        elif self.type == 'wrench':
            from bsk_controller.controllers.mpc_wrench import MpcWrench
            self.mpc = MpcWrench()
            self.control = np.zeros((self.mpc.nu, 1))
        elif self.type == 'follower_wrench':
            from bsk_controller.controllers.mpc_follower_wrench import MpcFollowerWrench
            self.mpc = MpcFollowerWrench(1) # Assuming 1 other agent (the leader)
            self.control = np.zeros((self.mpc.nu, 1))

        # Initialize others' odometry and predicted path
        self.leader = {
            "state": {
                "timestamp": np.zeros(1),
                "position": np.zeros(3),
                "velocity": np.zeros(3),
                "attitude": np.zeros(4),
                "angular_velocity": np.zeros(3),
            },
            "trajectory": {
                "timestamps": [],
                "position": [],
                "velocity": [],
                "attitude": [],
                "angular_velocity": [],
            }
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
        if self.use_hill:
            self.hill_trans_sub = self.create_subscription(
                HillRelStateMsgPayload,
                "bsk/out/hill_trans_state",
                self.hill_trans_callback,
                10
            )
            self.hill_rot_sub = self.create_subscription(
                AttGuidMsgPayload,
                "bsk/out/hill_rot_state",
                self.hill_rot_callback,
                10
            )
        else:
            self.sc_state_sub = self.create_subscription(
                SCStatesMsgPayload, 
                "bsk/out/sc_states", 
                self.sc_state_callback, 
                10
            )
        self.setpoint_pose_sub = self.create_subscription(
            PoseStamped,
            'bsk_mpc/setpoint_pose',
            self.setpoint_pose_callback,
            0
        )
        if self.type == 'follower_wrench':
            if self.use_hill:
                self.leader_hill_trans_sub = self.create_subscription(
                    HillRelStateMsgPayload,
                    f"/{self.name_leader}/bsk/out/hill_trans_state",
                    self.leader_hill_trans_callback,
                    10
                )
                self.leader_hill_rot_sub = self.create_subscription(
                    AttGuidMsgPayload,
                    f"/{self.name_leader}/bsk/out/hill_rot_state",
                    self.leader_hill_rot_callback,
                    10
                )
            else:
                self.leader_state_sub = self.create_subscription(
                    SCStatesMsgPayload,
                    f"/{self.name_leader}/bsk/out/sc_states",
                    self.leader_state_callback,
                    10
                )
            self.leader_traj_sub = [
                self.create_subscription(
                Path,
                f'/{self.name_leader}/bsk_mpc/predicted_path',
                self.leader_pred_callback,
                10)
            ]

        # Publishers
        self.predicted_path_pub = self.create_publisher(
            Path,
            'bsk_mpc/predicted_path',
            10)
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped,
            'bsk_mpc/vehicle_pose',
            10)
        self.vehicle_velocity_pub = self.create_publisher(
            Vector3Stamped,
            'bsk_mpc/vehicle_velocity',
            10)
        self.vehicle_angular_velocity_pub = self.create_publisher(
            Vector3Stamped,
            'bsk_mpc/vehicle_angular_velocity',
            10)
        self.ref_pose_pub = self.create_publisher(
            PoseStamped,
            'bsk_mpc/vehicle_pose_ref',
            10)
        self.ref_velocity_pub = self.create_publisher(
            Vector3Stamped,
            'bsk_mpc/vehicle_velocity_ref',
            10)
        self.ref_angular_velocity_pub = self.create_publisher(
            Vector3Stamped,
            'bsk_mpc/vehicle_angular_velocity_ref',
            10)
        if self.type == 'da':
            self.publisher_thruster_array_cmd = self.create_publisher(
                THRArrayCmdForceMsgPayload, 
                "bsk/in/thr_array_cmd_force", 
                10
            )
        elif self.type == 'wrench' or self.type == 'follower_wrench':
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
            self.get_logger().error(f"Unknown type: {self.type}. Use 'da' or 'wrench'.")
            return

    def sc_state_callback(self, msg: SCStatesMsgPayload):
        # position and velocity in inertial frame
        # attitude in body to inertial frame
        # angular velocity in body frame
        self.vehicle_local_position = msg.r_bn_n
        self.vehicle_local_velocity = msg.v_bn_n
        q_nb = MRP2quat(np.array(msg.sigma_bn), ref_quat=self.setpoint_attitude)
        self.vehicle_attitude = q_nb
        self.vehicle_angular_velocity = msg.omega_bn_b
    
    def hill_trans_callback(self, msg: HillRelStateMsgPayload):
        # position and velocity in Hill frame
        self.vehicle_local_position = msg.r_dc_h
        self.vehicle_local_velocity = msg.v_dc_h            

    def hill_rot_callback(self, msg: AttGuidMsgPayload):
        # attitude in body to Hill frame
        # angular velocity in body frame
        q_nb = MRP2quat(np.array(msg.sigma_br), ref_quat=self.setpoint_attitude)
        self.vehicle_attitude = q_nb
        self.vehicle_angular_velocity = msg.omega_br_b

    def leader_state_callback(self, msg: SCStatesMsgPayload):
        # Update leader spacecraft state
        self.leader["state"]["timestamp"] = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        self.leader["state"]["position"] = msg.r_bn_n
        self.leader["state"]["velocity"] = msg.v_bn_n
        q_nb = MRP2quat(np.array(msg.sigma_bn), ref_quat=self.vehicle_attitude)
        self.leader["state"]["attitude"] = q_nb
        self.leader["state"]["angular_velocity"] = msg.omega_bn_b

    def leader_hill_trans_callback(self, msg: HillRelStateMsgPayload):
        # position and velocity in Hill frame
        self.leader["state"]["position"] = msg.r_dc_h
        self.leader["state"]["velocity"] = msg.v_dc_h
    
    def leader_hill_rot_callback(self, msg: AttGuidMsgPayload):
        # attitude in body to Hill frame
        # angular velocity in body frame
        q_nb = MRP2quat(np.array(msg.sigma_br), ref_quat=self.vehicle_attitude)
        self.leader["state"]["attitude"] = q_nb
        self.leader["state"]["angular_velocity"] = msg.omega_br_b

    def leader_pred_callback(self, msg: Path):
        pred = self.leader['trajectory']

        # Reset stored prediction
        pred['timestamps'].clear()
        pred['position'].clear()
        pred['velocity'].clear()
        pred['attitude'].clear()
        pred['angular_velocity'].clear()

        for pose_stamped in msg.poses:
            t = pose_stamped.header.stamp
            timestamp_ns = 1e9 * t.sec + t.nanosec

            pos = np.array([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            ])
            att = np.array([
                pose_stamped.pose.orientation.w,
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z
            ])
            vel = np.array([0.0]*3)  # Placeholder for velocity
            ang_vel = np.array([0.0]*3)  # Placeholder for angular velocity

            pred['timestamps'].append(timestamp_ns)
            pred['position'].append(pos)
            pred['velocity'].append(vel)
            pred['attitude'].append(att)
            pred['angular_velocity'].append(ang_vel)

    def setpoint_pose_callback(self, msg):
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
        if np.dot(self.vehicle_attitude, self.setpoint_attitude) < 0:
            self.setpoint_attitude = -self.setpoint_attitude

        self.get_logger().info(f"Setpoint position: {self.setpoint_position}, attitude: {self.setpoint_attitude}")
        self.get_logger().info(f"Position: {self.vehicle_local_position}, Attitude: {self.vehicle_attitude}")
        self.received_first_setpoint = True

    def publish_reference(self, reference_position, reference_velocity, reference_attitude, reference_angular_rate):
        # Publish pose (position + attitude)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(reference_position[0])
        pose_msg.pose.position.y = float(reference_position[1])
        pose_msg.pose.position.z = float(reference_position[2])
        pose_msg.pose.orientation.w = float(reference_attitude[0])
        pose_msg.pose.orientation.x = float(reference_attitude[1])
        pose_msg.pose.orientation.y = float(reference_attitude[2])
        pose_msg.pose.orientation.z = float(reference_attitude[3])
        self.ref_pose_pub.publish(pose_msg)
        # Publish velocity
        twist_msg = Vector3Stamped()
        twist_msg.header.stamp = pose_msg.header.stamp
        twist_msg.header.frame_id = "map"
        twist_msg.vector.x = float(reference_velocity[0])
        twist_msg.vector.y = float(reference_velocity[1])
        twist_msg.vector.z = float(reference_velocity[2])
        self.ref_velocity_pub.publish(twist_msg)
        # Publish angular velocity
        angular_velocity_msg = Vector3Stamped()
        angular_velocity_msg.header.stamp = pose_msg.header.stamp
        angular_velocity_msg.header.frame_id = "map"
        angular_velocity_msg.vector.x = float(reference_angular_rate[0])
        angular_velocity_msg.vector.y = float(reference_angular_rate[1])
        angular_velocity_msg.vector.z = float(reference_angular_rate[2])
        self.ref_angular_velocity_pub.publish(angular_velocity_msg)

    def publish_current_state(self, position, velocity, attitude, angular_rate):
        # Publish current pose (position + attitude)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.w = float(attitude[0])
        pose_msg.pose.orientation.x = float(attitude[1])
        pose_msg.pose.orientation.y = float(attitude[2])
        pose_msg.pose.orientation.z = float(attitude[3])
        self.vehicle_pose_pub.publish(pose_msg)
        # Publish velocity
        velocity_msg = Vector3Stamped()
        velocity_msg.header.stamp = pose_msg.header.stamp
        velocity_msg.header.frame_id = "map"
        velocity_msg.vector.x = float(velocity[0])
        velocity_msg.vector.y = float(velocity[1])
        velocity_msg.vector.z = float(velocity[2])
        self.vehicle_velocity_pub.publish(velocity_msg)
        # Publish angular velocity
        angular_velocity_msg = Vector3Stamped()
        angular_velocity_msg.header.stamp = pose_msg.header.stamp
        angular_velocity_msg.header.frame_id = "map"
        angular_velocity_msg.vector.x = float(angular_rate[0])
        angular_velocity_msg.vector.y = float(angular_rate[1])
        angular_velocity_msg.vector.z = float(angular_rate[2])
        self.vehicle_angular_velocity_pub.publish(angular_velocity_msg)

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

    def publish_predicted_path(self, x_pred):
        now = self.get_clock().now()
        predicted_path_msg = Path()
        predicted_path_msg.header.stamp = now.to_msg()
        predicted_path_msg.header.frame_id = 'map'

        for i, predicted_state in enumerate(x_pred):
            # Calculate future time offset
            future_time = now + rclpy.duration.Duration(seconds=i * self.mpc.dt)

            # Create PoseStamped
            pose_stamped = self.vector2PoseMsg('map', predicted_state[0:3], predicted_state[6:10])
            pose_stamped.header.stamp = future_time.to_msg()

            predicted_path_msg.poses.append(pose_stamped)
        self.predicted_path_pub.publish(predicted_path_msg)

    def cmdloop_callback(self):
        if not self.received_first_setpoint:
            return
        
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

            # Publish current state
            self.publish_current_state(
                position=self.vehicle_local_position,
                velocity=self.vehicle_local_velocity,
                attitude=self.vehicle_attitude,
                angular_rate=self.vehicle_angular_velocity
            )
            # Publish reference state
            self.publish_reference(
                reference_position=self.setpoint_position,
                reference_velocity=self.setpoint_velocity,
                reference_attitude=self.setpoint_attitude,
                reference_angular_rate=self.setpoint_angular_velocity
            )
                                   
        elif self.type == 'follower_wrench':
            if self.leader["trajectory"]["timestamps"] == []:
                x_ref = np.concatenate((self.leader["state"]["position"] + self.setpoint_position,          # position
                                    np.zeros(3),                                                          # velocity
                                    self.setpoint_attitude,                                               # attitude
                                    np.zeros(3)), axis=0)                                                 # angular velocity   
                # Use current odometry of other spacecraft
                x_others = [
                    sample_other_path(
                        t0=self.get_clock().now().nanoseconds,
                        dt=self.mpc.dt,
                        Nx=self.mpc.Nx + 1,
                        t_other=[self.leader['state']['timestamp']],
                        pos_other=[self.leader['state']['position']],
                        vel_other=[self.leader['state']['velocity']],
                    )
                ]
                self.get_logger().warn("Leader trajectory is empty, using current leader state as reference.")
            else:
                # Use leader's predicted trajectory
                for i in range(len(self.leader["trajectory"]["timestamps"])):
                    x_ref = np.concatenate((self.leader["trajectory"]["position"][i] + self.setpoint_position,  # position
                                        self.leader["trajectory"]["velocity"][i],                               # velocity
                                        self.setpoint_attitude,          # attitude
                                        np.zeros(3)), axis=0)            # angular velocity
                # Use predicted paths of other spacecraft
                x_others = [
                    sample_other_path(
                        t0=self.get_clock().now().nanoseconds,
                        dt=self.mpc.dt,
                        Nx=self.mpc.Nx + 1,
                        t_other=self.leader['trajectory']['timestamps'],
                        pos_other=self.leader['trajectory']['position'],
                    )
                ]

            self.control, x_pred = self.mpc.get_input(x0, x_ref, x_others=x_others)

            # Publish current state
            self.publish_current_state(
                position=self.vehicle_local_position,
                velocity=self.vehicle_local_velocity,
                attitude=self.vehicle_attitude,
                angular_rate=self.vehicle_angular_velocity
            )
            # Publish reference state
            self.publish_reference(
                reference_position=self.setpoint_position + self.leader["state"]["position"],
                reference_velocity=self.setpoint_velocity + self.leader["state"]["velocity"],
                reference_attitude=self.setpoint_attitude,
                reference_angular_rate=self.setpoint_angular_velocity
            )

        else:
            raise ValueError(f'Invalid type: {self.type}')

        if self.type == 'da':
            self.publish_thruster_cmd(self.control)
        elif self.type == 'wrench' or self.type == 'follower_wrench':
            self.publish_wrench_cmd(self.control)

        # Publish predicted path
        self.publish_predicted_path(x_pred)
        

def main(args=None):
    rclpy.init(args=args)
    bsk_mpc = BskMpc()
    rclpy.spin(bsk_mpc)
    bsk_mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
