import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FollowerWaypointPublisher(Node):
    def __init__(self):
        # Declare 'is_sim' as a parameter that can be set via launch
        super().__init__('waypoint_publisher')
        self.declare_parameter('is_sim', False)
        is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, is_sim)])

        self.get_logger().info(f"Use sim time = {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")

        self.position = self.declare_parameter('position', [-1.0, 0.0, 0.0]).value
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
