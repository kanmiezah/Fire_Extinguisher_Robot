# camera_node/camera_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # /dev/video0

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            return

        self.timer = self.create_timer(0.1, self.publish_frame)  # 10Hz

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("Published image frame.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
