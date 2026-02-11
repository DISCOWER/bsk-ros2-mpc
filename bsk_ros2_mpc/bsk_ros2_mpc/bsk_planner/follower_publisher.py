#!/usr/bin/env python
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FollowerWaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Setup parameters
        self._setup_parameters()

        self.get_logger().info(f"Use sim time = {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")

        self.publisher = self.create_publisher(
            PoseStamped, "bsk_mpc/setpoint_pose", 10)
        self.timer_period = 0.1  # 10 Hz

        self.msg = PoseStamped()
        self.msg.header.frame_id = "map"
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.pose.position.x = self.position[0]
        self.msg.pose.position.y = self.position[1]
        self.msg.pose.position.z = self.position[2]
        self.msg.pose.orientation.w = 1.0
        self.msg.pose.orientation.x = 0.0
        self.msg.pose.orientation.y = 0.0
        self.msg.pose.orientation.z = 0.0

        self.timer = self.create_timer(self.timer_period, self.publish_waypoint)

    def _setup_parameters(self):
        """Configure ROS parameters for port settings."""
        self.declare_parameter('position', [-1.0, 0.0, 0.0])

        # use_sim_time is automatically declared by ROS2, just get its value
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f"Use sim time: {self.use_sim_time}")
        self.position = self.get_parameter('position').get_parameter_value().double_array_value

    def publish_waypoint(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

def main():
    rclpy.init()
    node = FollowerWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
