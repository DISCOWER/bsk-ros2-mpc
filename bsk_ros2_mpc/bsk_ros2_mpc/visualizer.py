__author__ = "Elias Krantz"
__contact__ = "eliaskra@kth.se"

import numpy as np

from .tools.utils import MRP2quat
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from bsk_msgs.msg import HillRelStateMsgPayload, AttGuidMsgPayload, SCStatesMsgPayload
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from mpc_msgs.srv import SetPose

class BskMpcVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.declare_parameter('use_hill', True)
        self.use_hill = self.get_parameter('use_hill').get_parameter_value().bool_value
        self.get_logger().info(f"Use Hill frame: {self.use_hill}")

        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        if self.use_hill:
            self.hill_trans_sub = self.create_subscription(
                HillRelStateMsgPayload,
                "bsk/out/hill_trans_state",
                self.hill_trans_callback,
                qos_profile
            )
            self.hill_rot_sub = self.create_subscription(
                AttGuidMsgPayload,
                "bsk/out/hill_rot_state",
                self.hill_rot_callback,
                qos_profile
            )
        else:
            self.sc_state_sub = self.create_subscription(
                SCStatesMsgPayload, 
                "bsk/out/sc_states", 
                self.sc_state_callback, 
                qos_profile
            )
        self.setpoint_pose_srv = self.create_service(
            SetPose,
            'set_pose',
            self.add_setpoint_pose_callback
        )

        # Publishers
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, f"bsk_visualizer/vehicle_pose", 10
        )
        self.vehicle_vel_pub = self.create_publisher(
            Marker, f"bsk_visualizer/vehicle_velocity", 10
        )
        self.vehicle_path_pub = self.create_publisher(
            Path, f"bsk_visualizer/vehicle_path", 10
        )
        self.setpoint_path_pub = self.create_publisher(
            Path, f"bsk_visualizer/setpoint_path", 10
        )

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()
        self.setpoint_attitude = np.array([1.0, 0.0, 0.0, 0.0])

        # trail size
        self.trail_size = 1000

        # time stamp for the last local position update received on ROS2 topic
        self.last_local_pos_update = 0.0
        # time after which existing path is cleared upon receiving new
        # local position ROS2 message
        self.declare_parameter("path_clearing_timeout", -1.0)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vector2PoseMsg(self, frame_id, position, attitude):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.orientation.w = attitude[0]
        pose_msg.pose.orientation.x = attitude[1]
        pose_msg.pose.orientation.y = attitude[2]
        pose_msg.pose.orientation.z = attitude[3]
        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]
        return pose_msg

    def sc_state_callback(self, msg):
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (self.get_clock().now() / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

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

    def add_setpoint_pose_callback(self, request, response):
        # Extract and normalize quaternion
        new_attitude = np.array([
            request.pose.orientation.w,
            request.pose.orientation.x,
            request.pose.orientation.y,
            request.pose.orientation.z
        ])
        norm = np.linalg.norm(new_attitude)
        if norm > 0:
            new_attitude /= norm
        if np.dot(self.vehicle_attitude, new_attitude) < 0:
            self.setpoint_attitude = -new_attitude
        else:
            self.setpoint_attitude = new_attitude
        return response

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position = msg.position

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = "arrow"
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1*0.3
        msg.scale.y = 0.2*0.3
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def append_vehicle_path(self, msg):
        self.vehicle_path_msg.poses.append(msg)
        if len(self.vehicle_path_msg.poses) > self.trail_size:
            del self.vehicle_path_msg.poses[0]

    def append_setpoint_path(self, msg):
        self.setpoint_path_msg.poses.append(msg)
        if len(self.setpoint_path_msg.poses) > self.trail_size:
            del self.setpoint_path_msg.poses[0]

    def cmdloop_callback(self):
        vehicle_pose_msg = self.vector2PoseMsg(
            "map", self.vehicle_local_position, self.vehicle_attitude
        )
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        # Publish time history of the vehicle path
        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.append_vehicle_path(vehicle_pose_msg)
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Publish time history of the vehicle path
        setpoint_pose_msg = self.vector2PoseMsg("odom", self.vehicle_local_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.append_setpoint_path(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        # Publish arrow markers for velocity
        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_vel_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    bsk_mpc_visualizer = BskMpcVisualizer()

    rclpy.spin(bsk_mpc_visualizer)

    bsk_mpc_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
