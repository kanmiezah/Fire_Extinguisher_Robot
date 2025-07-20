# camera_node/camera_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Declare and get parameters
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)

        camera_index = self.get_parameter("camera_index").value
        frame_width = self.get_parameter("frame_width").value
        frame_height = self.get_parameter("frame_height").value

        # Set up publisher and bridge
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            return

        self.get_logger().info(f"Camera {camera_index} opened at {frame_width}x{frame_height}")
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10Hz

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame.")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().debug("Published image frame.")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
