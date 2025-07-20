import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import serial
import time
from math import sin, cos, pi

class OdometryNode(Node):
    def __init__(self):
        super().__init__('diff_drive_odometry_node')

        # Robot parameters (update with your actual values)
        self.TICKS_PER_REV = 500
        self.WHEEL_RADIUS = 0.03  # meters
        self.BASE_WIDTH = 0.15    # meters
        self.TICK_DISTANCE = 2 * pi * self.WHEEL_RADIUS / self.TICKS_PER_REV

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.prev_left = 0
        self.prev_right = 0
        self.first_reading = True

        # Serial port to Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        time.sleep(2)  # wait for Arduino to reset

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.1, self.update_odometry)  # 10 Hz
        self.get_logger().info('Diff drive odometry node started')

    def update_odometry(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if not line.startswith('L:'):
            return

        try:
            parts = line.split()
            left_ticks = int(parts[0].split(':')[1])
            right_ticks = int(parts[1].split(':')[1])
        except Exception as e:
            self.get_logger().warn(f'Invalid encoder line: {line}')
            return

        if self.first_reading:
            self.prev_left = left_ticks
            self.prev_right = right_ticks
            self.first_reading = False
            return

        # Calculate tick differences
        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right

        self.prev_left = left_ticks
        self.prev_right = right_ticks

        # Convert to distance
        dist_left = delta_left * self.TICK_DISTANCE
        dist_right = delta_right * self.TICK_DISTANCE
        delta_dist = (dist_left + dist_right) / 2.0
        delta_th = (dist_right - dist_left) / self.BASE_WIDTH

        # Update pose
        self.x += delta_dist * cos(self.th + delta_th / 2.0)
        self.y += delta_dist * sin(self.th + delta_th / 2.0)
        self.th += delta_th

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        self.odom_pub.publish(odom)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
