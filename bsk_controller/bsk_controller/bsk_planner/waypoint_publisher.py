import rclpy
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
        super().__init__('waypoint_publisher')

        self.period = self.declare_parameter('period', 20.0).value

        self.publisher = self.create_publisher(
            PoseStamped, "bsk_mpc/setpoint_pose", 10)
        
        self.timer_period = 0.05  # 20 Hz
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.timer_period, self.publish_waypoint)
        
        self.waypoints = [
            [1.5, 0.0, 0.0],
            [3.0, 1.0, 0.0],
            [3.0, -1.0, 0.0]
        ]

    def publish_waypoint(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9  # seconds
        idx = int(elapsed // self.period) % 3
        # yaw = (idx % 4) * (math.pi / 2.0)  # 0, 90, 180, 270 deg
        yaw = 0.0

        pos = self.waypoints[idx]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        quat = euler_to_quaternion(0.0, 0.0, yaw)
        msg.pose.orientation.x = quat['x']
        msg.pose.orientation.y = quat['y']
        msg.pose.orientation.z = quat['z']
        msg.pose.orientation.w = quat['w']

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
