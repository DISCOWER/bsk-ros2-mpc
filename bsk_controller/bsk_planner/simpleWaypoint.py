import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

# Set these variables directly in the script (angles in degrees)
namespace = "bskSat0"
x = -5.0
y = 0.0
z = 0.0
roll_deg = 0.0   # degrees
pitch_deg = 0.0  # degrees
yaw_deg = 0.0    # degrees

def euler_to_quaternion(roll, pitch, yaw):
    # Convert euler angles (in radians) to quaternion
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

def main():
    rclpy.init()

    # Create node
    node = Node('simple_waypoint_sender')

    # Publisher
    topic = f"/{namespace}/bsk_mpc/setpoint_pose"
    pub = node.create_publisher(PoseStamped, topic, 10)

    # Create PoseStamped message
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    # Convert degrees to radians
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    # Convert euler to quaternion
    quat = euler_to_quaternion(roll, pitch, yaw)
    msg.pose.orientation.x = quat['x']
    msg.pose.orientation.y = quat['y']
    msg.pose.orientation.z = quat['z']
    msg.pose.orientation.w = quat['w']

    # Publish once
    pub.publish(msg)
    node.get_logger().info(f"Published waypoint to {topic}")

    # Sleep briefly to ensure message is sent
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()