import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import os


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        # Declare ROS 2 parameter for model path
        self.declare_parameter('model_path', os.path.expanduser('~/fxb_ws/FYP-AI-Model/best.pt'))
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Load YOLOv8 model
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f"✅ Loaded YOLOv8 model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load PyTorch model: {e}")
            raise

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(PoseArray, '/fire_targets', 10)

        self.score_threshold = 0.4
        self.target_class_id = 0  # fire

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame)[0]  # Run YOLOv8 inference
            self.process_detections(results, frame.shape[1], frame.shape[0], msg.header)
        except Exception as e:
            self.get_logger().error(f"YOLOv8 Inference failed: {e}")

    def process_detections(self, results, image_width, image_height, header: Header):
        poses = PoseArray()
        poses.header = header

        for det in results.boxes:
            conf = float(det.conf)
            class_id = int(det.cls)

            if conf < self.score_threshold or class_id != self.target_class_id:
                continue

            # Get bounding box center
            x1, y1, x2, y2 = det.xyxy[0]
            x_center = float((x1 + x2) / 2)
            y_center = float((y1 + y2) / 2)

            pose = Pose()
            pose.position.x = x_center / image_width
            pose.position.y = y_center / image_height
            pose.position.z = 0.0
            poses.poses.append(pose)

        self.publisher.publish(poses)
        self.get_logger().info(f"🔥 Published {len(poses.poses)} fire detections.")


def main(args=None):
    rclpy.init(args=args)
    node = YoloInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
