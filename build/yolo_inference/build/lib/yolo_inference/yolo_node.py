
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import os


class YoloInferenceNode(Node):
    def __init__(self):
        super().__init__('yolo_inference_node')

        # Subscriptions and Publishers
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(PoseArray, '/fire_targets', 10)

        # Load ONNX model
        model_path = os.path.expanduser('~/fxb_ws/FYP-AI-Model/best.onnx')  # Update path to your ONNX model
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.score_threshold = 0.4
        self.target_class_id = 0  # fire class index

        self.get_logger().info(f"Loaded YOLO model: {model_path}")

    def preprocess(self, image):
        img = cv2.resize(image, (self.input_shape[2], self.input_shape[3]))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        img = np.expand_dims(img, axis=0)
        return img

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            input_tensor = self.preprocess(frame)
            outputs = self.session.run(None, {self.input_name: input_tensor})[0]
            self.process_detections(outputs, frame.shape[1], frame.shape[0])
        except Exception as e:
            self.get_logger().error(f"YOLO Inference failed: {e}")

    def process_detections(self, detections, image_width, image_height):
        poses = PoseArray()

        for det in detections:
            if len(det) < 6:
                continue  # Skip invalid detections

            x_center, y_center, w, h, conf, class_id = det[:6]
            if conf < self.score_threshold or int(class_id) != self.target_class_id:
                continue

            pose = Pose()
            pose.position.x = float(x_center / image_width)
            pose.position.y = float(y_center / image_height)
            pose.position.z = 0.0
            poses.poses.append(pose)

        self.publisher.publish(poses)
        self.get_logger().info(f"Published {len(poses.poses)} fire detections.")


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
