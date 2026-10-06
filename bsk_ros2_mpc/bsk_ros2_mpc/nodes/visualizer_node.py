#!/usr/bin/env python
import numpy as np
from ..tools.utils import MRP2quat
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from bsk_msgs.msg import HillRelStateMsgPayload, AttGuidMsgPayload, SCStatesMsgPayload
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

class BskMpcVisualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        self.declare_parameter('use_hill', True)
        self.use_hill = self.get_parameter('use_hill').get_parameter_value().bool_value
        self.get_logger().info(f"Use Hill frame: {self.use_hill}")
        self.declare_parameter('agents', '')
        agents = self.get_parameter('agents').get_parameter_value().string_value
        self.agents = agents.split() if agents else []
        self.agent_states = {
            name: {
                'position': np.zeros(3),
                'attitude': np.array([1.0, 0.0, 0.0, 0.0]),
                'seen': False,
                'vehicle_path': Path(),
                'setpoint_path': Path(),
                'predicted_path': Path(),
                'setpoint_pose': PoseStamped(),
                'setpoint_seen': False,
                'last_update': 0.0,
                'labels_dirty': True,
            }
            for name in self.agents
        }
        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.state_subscriptions = []
        for name in self.agents:
            if self.use_hill:
                self.state_subscriptions.extend([
                    self.create_subscription(
                        HillRelStateMsgPayload,
                        f'/{name}/bsk/out/hill_trans_state',
                        lambda msg, n=name: self.hill_trans_callback(msg, n),
                        qos_profile,
                    ),
                    self.create_subscription(
                        AttGuidMsgPayload,
                        f'/{name}/bsk/out/hill_rot_state',
                        lambda msg, n=name: self.hill_rot_callback(msg, n),
                        qos_profile,
                    ),
                ])
            else:
                self.state_subscriptions.append(
                    self.create_subscription(
                        SCStatesMsgPayload,
                        f'/{name}/bsk/out/sc_states',
                        lambda msg, n=name: self.sc_state_callback(msg, n),
                        qos_profile,
                    )
                )

        self.setpoint_subscriptions = [
            self.create_subscription(
                PoseStamped,
                f'/{name}/bsk_mpc/vehicle_pose_ref',
                lambda msg, n=name: self.setpoint_pose_callback(msg, n),
                10,
            )
            for name in self.agents
        ]
        self.predicted_subscriptions = [
            self.create_subscription(
                Path,
                f'/{name}/bsk_mpc/predicted_path',
                lambda msg, n=name: self.predicted_path_callback(msg, n),
                10,
            )
            for name in self.agents
        ]

        self.pose_pubs = {
            name: self.create_publisher(PoseStamped, f'bsk_visualizer/{name}/pose', 10)
            for name in self.agents
        }
        self.setpoint_pose_pubs = {
            name: self.create_publisher(PoseStamped, f'bsk_visualizer/{name}/setpoint_pose', 10)
            for name in self.agents
        }
        label_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.labels_pub = self.create_publisher(MarkerArray, 'bsk_visualizer/labels', label_qos)
        self.radius_pubs = {
            name: self.create_publisher(Marker, f'bsk_visualizer/{name}/radii', 10)
            for name in self.agents
        }
        self.vehicle_path_pubs = {
            name: self.create_publisher(Path, f'bsk_visualizer/{name}/vehicle_path', 10)
            for name in self.agents
        }
        self.setpoint_path_pubs = {
            name: self.create_publisher(Path, f'bsk_visualizer/{name}/setpoint_path', 10)
            for name in self.agents
        }
        self.predicted_path_pubs = {
            name: self.create_publisher(Path, f'bsk_visualizer/{name}/predicted_path', 10)
            for name in self.agents
        }

        # trail size
        self.trail_size = 1000
        self.labels_publish_period = 0.2
        self.last_labels_publish = None

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

    def update_agent_state(self, name):
        state = self.agent_states[name]
        now = self.get_clock().now().nanoseconds / 1e9
        timeout = self.get_parameter('path_clearing_timeout').get_parameter_value().double_value
        if timeout >= 0 and state['last_update'] > 0 and now - state['last_update'] > timeout:
            state['vehicle_path'].poses.clear()
        state['last_update'] = now
        state['seen'] = True
        state['labels_dirty'] = True

    def sc_state_callback(self, msg, name):
        state = self.agent_states[name]
        state['position'] = np.array(msg.r_bn_n)
        state['attitude'] = MRP2quat(
            np.array(msg.sigma_bn),
            ref_quat=state['attitude'],
        )
        self.update_agent_state(name)

    def hill_trans_callback(self, msg, name):
        self.agent_states[name]['position'] = np.array(msg.r_dc_h)
        self.update_agent_state(name)

    def hill_rot_callback(self, msg, name):
        state = self.agent_states[name]
        state['attitude'] = MRP2quat(
            np.array(msg.sigma_br),
            ref_quat=state['attitude'],
        )
        self.update_agent_state(name)

    def setpoint_pose_callback(self, msg, name):
        self.agent_states[name]['setpoint_pose'] = msg
        self.agent_states[name]['setpoint_seen'] = True
        self.agent_states[name]['labels_dirty'] = True

    def predicted_path_callback(self, msg, name):
        self.agent_states[name]['predicted_path'] = msg
        self.predicted_path_pubs[name].publish(msg)

    def create_collision_radius_marker(self, state):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "collision_radius"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = float(state['position'][0])
        marker.pose.position.y = float(state['position'][1])
        marker.pose.position.z = float(state['position'][2])
        return marker

    def create_label_marker(self, position, text, color, marker_id, action=Marker.ADD):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = action
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.13
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.text = text
        return marker

    def publish_labels(self):
        labels = MarkerArray()
        for index, (name, state) in enumerate(self.agent_states.items()):
            pose_marker_id = 2 * index
            setpoint_marker_id = pose_marker_id + 1
            if state['seen']:
                labels.markers.append(
                    self.create_label_marker(
                        state['position'],
                        f'{name}_pose',
                        (1.0, 0.1, 0.0),
                        pose_marker_id,
                    )
                )
            else:
                labels.markers.append(
                    self.create_label_marker(
                        (0.0, 0.0, 0.0),
                        '',
                        (0.0, 0.0, 0.0),
                        pose_marker_id,
                        Marker.DELETE,
                    )
                )
            if state['setpoint_seen']:
                setpoint = state['setpoint_pose'].pose.position
                labels.markers.append(
                    self.create_label_marker(
                        (setpoint.x, setpoint.y, setpoint.z + 0.3),
                        f'{name}_setpoint',
                        (0.0, 0.0, 1.0),
                        setpoint_marker_id,
                    )
                )
            else:
                labels.markers.append(
                    self.create_label_marker(
                        (0.0, 0.0, 0.0),
                        '',
                        (0.0, 0.0, 0.0),
                        setpoint_marker_id,
                        Marker.DELETE,
                    )
                )
        self.labels_pub.publish(labels)
        for state in self.agent_states.values():
            state['labels_dirty'] = False

    def cmdloop_callback(self):
        for name, state in self.agent_states.items():
            if not state['seen']:
                continue

            pose_msg = self.vector2PoseMsg(
                "map", state['position'], state['attitude']
            )
            self.pose_pubs[name].publish(pose_msg)
            self.radius_pubs[name].publish(self.create_collision_radius_marker(state))

            vehicle_path = state['vehicle_path']
            vehicle_path.header = pose_msg.header
            vehicle_path.poses.append(pose_msg)
            if len(vehicle_path.poses) > self.trail_size:
                del vehicle_path.poses[0]
            self.vehicle_path_pubs[name].publish(vehicle_path)

            if state['setpoint_seen']:
                setpoint_pose = state['setpoint_pose']
                self.setpoint_pose_pubs[name].publish(setpoint_pose)
            else:
                setpoint_pose = pose_msg
            setpoint_path = state['setpoint_path']
            setpoint_path.header = setpoint_pose.header
            setpoint_path.poses.append(setpoint_pose)
            if len(setpoint_path.poses) > self.trail_size:
                del setpoint_path.poses[0]
            self.setpoint_path_pubs[name].publish(setpoint_path)

        labels_dirty = any(state['labels_dirty'] for state in self.agent_states.values())
        now = self.get_clock().now().nanoseconds / 1e9
        labels_due = (
            self.last_labels_publish is None
            or now - self.last_labels_publish >= self.labels_publish_period
        )
        if labels_dirty and labels_due:
            self.publish_labels()
            self.last_labels_publish = now

def main(args=None):
    rclpy.init(args=args)
    bsk_mpc_visualizer = BskMpcVisualizer()
    rclpy.spin(bsk_mpc_visualizer)
    bsk_mpc_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
