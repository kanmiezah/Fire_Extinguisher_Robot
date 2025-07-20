import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math
import time

from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Declare ROS parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base', 0.15)
        self.declare_parameter('ticks_per_rev', 3200)

        # Retrieve parameters
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().integer_value

        # Connect to serial port
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2)  # Allow serial to initialize
            self.get_logger().info(f"Connected to serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to {port}: {e}")
            exit(1)

        # Initialize odometry state
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

        # Quaternion from theta
        q = quaternion_from_euler(0, 0, self.theta)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
