import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time

class FireConfidenceNode(Node):
    def __init__(self):
        super().__init__('fire_confidence_node')

        # Declare and validate parameter
        self.declare_parameter('max_detections', 5)
        self.max_detections = self.get_parameter('max_detections').get_parameter_value().integer_value

        if self.max_detections <= 0:
            self.get_logger().warn("max_detections must be > 0. Resetting to 1.")
            self.max_detections = 1

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            PoseArray,
            '/fire_targets',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Float32, '/fire_confidence', 10)

        self.get_logger().info('🔥 Fire Confidence Node started. Listening to /fire_targets')

    def callback(self, msg: PoseArray):
        num_fires = len(msg.poses)
        confidence = min(num_fires / float(self.max_detections), 1.0)

        # Publish confidence score
        self.publisher.publish(Float32(data=confidence))

        # Debug logs
        timestamp = self.get_clock().now().to_msg()
        if confidence == 0.0:
            self.get_logger().info(f"[{timestamp.sec}.{timestamp.nanosec}] No fire detected.")
        else:
            self.get_logger().info(
                f"[{timestamp.sec}.{timestamp.nanosec}] Fire confidence: {confidence:.2f} ({num_fires} detections)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = FireConfidenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
