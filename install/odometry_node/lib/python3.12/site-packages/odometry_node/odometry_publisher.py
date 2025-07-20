import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math
import time


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Serial port config
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
        time.sleep(2)  # Allow serial to initialize

        # Robot parameters
        self.wheel_radius = 0.03  # meters
        self.wheel_base = 0.15    # meters
        self.ticks_per_rev = 3200

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = 0
        self.last_right = 0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_odometry)

    def read_encoders(self):
        try:
            line = self.serial_port.readline().decode().strip()
            if line.startswith('L:'):
                parts = line.split()
                left = int(parts[0].split(':')[1])
                right = int(parts[1].split(':')[1])
                return left, right
        except Exception as e:
            self.get_logger().warn(f"Failed to parse serial: {e}")
        return None, None

    def update_odometry(self):
        left, right = self.read_encoders()
        if left is None or right is None:
            return

        # Time delta
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        # Delta ticks
        d_left = left - self.last_left
        d_right = right - self.last_right
        self.last_left = left
        self.last_right = right

        # Distance traveled per wheel
        dist_left = 2 * math.pi * self.wheel_radius * d_left / self.ticks_per_rev
        dist_right = 2 * math.pi * self.wheel_radius * d_right / self.ticks_per_rev

        # Robot motion
        delta_s = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base

        self.theta += delta_theta
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

        # Velocities
        vx = delta_s / dt
        vth = delta_theta / dt

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()