import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = {}
    q['w'] = cr * cp * cy + sr * sp * sy
    q['x'] = sr * cp * cy - cr * sp * sy
    q['y'] = cr * sp * cy + sr * cp * sy
    q['z'] = cr * cp * sy - sr * sp * cy
    return q

class WaypointPublisher(Node):
    def __init__(self):
        # Declare 'is_sim' as a parameter that can be set via launch
        super().__init__('waypoint_publisher')
        self.declare_parameter('is_sim', False)
        is_sim = self.get_parameter('is_sim').get_parameter_value().bool_value
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, is_sim)])

        self.get_logger().info(f"Use sim time = {self.get_parameter('use_sim_time').get_parameter_value().bool_value}")

        # Check if use_sim_time is enabled
        if self.get_parameter('use_sim_time').get_parameter_value().bool_value:
            self.get_logger().info("Using simulation time, waiting for /clock...")
            self.wait_for_clock()

        self.period = self.declare_parameter('period', 20.0).value

        self.publisher = self.create_publisher(
            PoseStamped, "bsk_mpc/setpoint_pose", 10)
        
        self.timer_period = 0.05  # 20 Hz
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.publish_waypoint)
        
        self.waypoints = [
            [1.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [3.0, 0.7, 0.0, 1.0, 0.0, 0.0, 0.0],
            [3.0, 0.7, 0.0, 0.7071068, 0.0, 0.0, 0.7071068],
            [3.0, -0.7, 0.0, 0.7071068, 0.0, 0.0, 0.7071068],
            [1.5, 0.0, 0.0, 0.7071068, 0.0, 0.0, 0.7071068],
        ]

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

    def publish_waypoint(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds
        idx = int(elapsed // self.period) % len(self.waypoints)

        waypoint = self.waypoints[idx]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = waypoint[0]
        msg.pose.position.y = waypoint[1]
        msg.pose.position.z = waypoint[2]

        msg.pose.orientation.w = waypoint[3]
        msg.pose.orientation.x = waypoint[4]
        msg.pose.orientation.y = waypoint[5]
        msg.pose.orientation.z = waypoint[6]

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
