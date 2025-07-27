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
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from .tools.utils import sample_other_path

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleAngularVelocity
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleTorqueSetpoint
from px4_msgs.msg import VehicleThrustSetpoint

DATA_VALIDITY_STREAM = 0.5 # seconds, threshold for (pos,att,vel) messages
DATA_VALIDITY_STATUS = 2.0 # seconds, threshold for status message

class BskMpc(Node):
    def __init__(self):
        super().__init__('bsk_mpc')
        
        # Get type
        self.type = self.declare_parameter('type', 'da').value

        # Get name of leader spacecraft
        self.name_leader = self.declare_parameter('name_leader', '').value

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # Setup publishers and subscribers
        self.set_publishers_subscribers(qos_profile_pub, qos_profile_sub)

        timer_period_cmd = 0.2  # seconds
        self.timer_cmd = self.create_timer(timer_period_cmd, self.cmdloop_callback)

        timer_offboard = 0.1  # seconds
        self.timer_offboard = self.create_timer(timer_offboard, self.offboard_control_mode_callback)

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

    def set_publishers_subscribers(self, qos_profile_pub, qos_profile_sub):
        # Subscribers
        self.status_sub_v1 = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            'fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile_sub)
        self.angular_vel_sub = self.create_subscription(
            VehicleAngularVelocity,
            'fmu/out/vehicle_angular_velocity',
            self.vehicle_angular_velocity_callback,
            qos_profile_sub)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile_sub)
        self.setpoint_pose_sub = self.create_subscription(
            PoseStamped,
            'bsk_mpc/setpoint_pose',
            self.setpoint_pose_callback,
            0
        )
        if self.type == 'follower_wrench':
            self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                f'{self.name_leader}/fmu/out/vehicle_attitude',
                self.leader_attitude_callback,
                qos_profile_sub)
            self.angular_vel_sub = self.create_subscription(
                VehicleAngularVelocity,
                f'{self.name_leader}/fmu/out/vehicle_angular_velocity',
                self.leader_angular_velocity_callback,
                qos_profile_sub)
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                f'{self.name_leader}/fmu/out/vehicle_local_position',
                self.leader_local_position_callback,
                qos_profile_sub)
            self.leader_traj_sub = [
                self.create_subscription(
                Path,
                f'/{self.name_leader}/bsk_mpc/predicted_path',
                self.leader_pred_callback,
                10)
            ]

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos_profile_pub)
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
            self.publisher_direct_actuator = self.create_publisher(
                ActuatorMotors,
                'fmu/in/actuator_motors',
                qos_profile_pub)
        elif self.type == 'wrench' or self.type == 'follower_wrench':
            self.publisher_thrust_setpoint = self.create_publisher(
                VehicleThrustSetpoint,
                'fmu/in/vehicle_thrust_setpoint',
                qos_profile_pub)
            self.publisher_torque_setpoint = self.create_publisher(
                VehicleTorqueSetpoint,
                'fmu/in/vehicle_torque_setpoint',
                qos_profile_pub)
        else:
            self.get_logger().error(f"Unknown type: {self.type}. Use 'da' or 'wrench'.")
            return

    def vehicle_attitude_callback(self, msg):
        self.vehicle_attitude_timestamp = Clock().now().nanoseconds / 1e9
        # NED-> ENU transformation
        # Receives quaternion in NED frame as (qw, qx, qy, qz)
        q_enu = 1/np.sqrt(2) * np.array([msg.q[0] + msg.q[3], msg.q[1] + msg.q[2], msg.q[1] - msg.q[2], msg.q[0] - msg.q[3]])
        q_enu /= np.linalg.norm(q_enu)
        self.vehicle_attitude = q_enu.astype(float)

    def vehicle_local_position_callback(self, msg):
        # NED-> ENU transformation
        self.vehicle_local_position_timestamp = Clock().now().nanoseconds / 1e9
        self.vehicle_local_position[0] = msg.y
        self.vehicle_local_position[1] = msg.x
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vy
        self.vehicle_local_velocity[1] = msg.vx
        self.vehicle_local_velocity[2] = -msg.vz

    def vehicle_angular_velocity_callback(self, msg):
        # NED-> ENU transformation
        self.vehicle_angular_velocity_timestamp = Clock().now().nanoseconds / 1e9
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]

    def vehicle_status_callback(self, msg):
        # print("NAV_STATUS: ", msg.nav_state)
        # print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.vehicle_status_timestamp = Clock().now().nanoseconds / 1e9
        self.nav_state = msg.nav_state

    def leader_attitude_callback(self, msg):
        # NED-> ENU transformation
        # Receives quaternion in NED frame as (qw, qx, qy, qz)
        q_enu = 1/np.sqrt(2) * np.array([msg.q[0] + msg.q[3], msg.q[1] + msg.q[2], msg.q[1] - msg.q[2], msg.q[0] - msg.q[3]])
        q_enu /= np.linalg.norm(q_enu)
        self.leader["state"]["attitude"] = q_enu.astype(float)

    def leader_local_position_callback(self, msg):
        # NED-> ENU transformation
        self.leader["state"]["position"][0] = msg.y
        self.leader["state"]["position"][1] = msg.x
        self.leader["state"]["position"][2] = -msg.z
        self.leader["state"]["velocity"][0] = msg.vy
        self.leader["state"]["velocity"][1] = msg.vx
        self.leader["state"]["velocity"][2] = -msg.vz

    def leader_angular_velocity_callback(self, msg):
        # NED-> ENU transformation
        self.leader["state"]["angular_velocity"][0] = msg.xyz[0]
        self.leader["state"]["angular_velocity"][1] = -msg.xyz[1]
        self.leader["state"]["angular_velocity"][2] = -msg.xyz[2]

    def leader_pred_callback(self, msg: Path, namespace):
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
        actuator_outputs_msg = ActuatorMotors()
        actuator_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # Normalize thrust values w.r.t. max thrust
        thrust = u[0, :]

        # Generate actuator outputs dynamically
        thrust_command = []
        for t in thrust:
            thrust_command.extend([max(t, 0.0), max(-t, 0.0)])
        thrust_command = np.clip(np.array(thrust_command, dtype=np.float32), 0.0, 1.0)

        actuator_outputs_msg.control[:len(thrust_command)] = thrust_command
        self.publisher_direct_actuator.publish(actuator_outputs_msg)

    def publish_wrench_cmd(self, u):
        # u is [F, T] in FLU frame

        # The PX4 uses normalized wrench input. Scaling with respect to the maximum force and torque.
        F_scaling = 1/(2 * 1.5)
        T_scaling = 1/(4 * 0.12 * 1.5)
        u[0:3] *= F_scaling
        u[3:6] *= T_scaling

        thrust_outputs_msg = VehicleThrustSetpoint()
        thrust_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        torque_outputs_msg = VehicleTorqueSetpoint()
        torque_outputs_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        thrust_outputs_msg.xyz = [u[0], -u[1], -u[2]]
        torque_outputs_msg.xyz = [u[3], -u[4], -u[5]]

        self.publisher_thrust_setpoint.publish(thrust_outputs_msg)
        self.publisher_torque_setpoint.publish(torque_outputs_msg)

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
    
    def check_data_validity(self):
        current_time = Clock().now().nanoseconds / 1e9

        # Check if the data is valid based on the timestamps
        if (current_time - self.vehicle_attitude_timestamp > DATA_VALIDITY_STREAM or
            current_time - self.vehicle_local_position_timestamp > DATA_VALIDITY_STREAM or
            current_time - self.vehicle_angular_velocity_timestamp > DATA_VALIDITY_STREAM):
            self.get_logger().warn("Vehicle attitude, position, or angular velocity data is too old. Skipping offboard control...")
            return False

        if (current_time - self.vehicle_status_timestamp > DATA_VALIDITY_STATUS):
            self.get_logger().warn("Vehicle status data is too old. Skipping offboard control...")
            return False

        return True

    def offboard_control_mode_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.direct_actuator = False
        if self.type == 'da':
            offboard_msg.direct_actuator = True
        elif self.type == 'wrench' or self.type == 'follower_wrench':
            offboard_msg.thrust_and_torque = True
        self.publisher_offboard_mode.publish(offboard_msg)

    def cmdloop_callback(self):
        if not self.received_first_setpoint:
            return
        
        # Check data validity
        if not self.check_data_validity():
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

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
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
