import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float32

class FireConfidenceNode(Node):
    def __init__(self):
        super().__init__('fire_confidence_node')

        # Declare parameter for maximum expected detections
        self.declare_parameter('max_detections', 5)
        self.max_detections = self.get_parameter('max_detections').get_parameter_value().integer_value

        self.subscription = self.create_subscription(
            PoseArray,
            '/fire_targets',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(Float32, '/fire_confidence', 10)

        self.get_logger().info('🔥 Fire Confidence Node started.')

    def callback(self, msg: PoseArray):
        num_fires = len(msg.poses)
        confidence = min(num_fires / float(self.max_detections), 1.0)
        self.publisher.publish(Float32(data=confidence))

        # Optional logging logic
        if confidence == 0.0:
            self.get_logger().info("No fire detected.")
        elif confidence > 0.3:
            self.get_logger().info(f'Fire confidence: {confidence:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FireConfidenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
