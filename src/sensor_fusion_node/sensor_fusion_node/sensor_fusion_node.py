# sensor_fusion_node/sensor_fusion_node/sensor_fusion_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.fire_sub = self.create_subscription(Float32, '/fire_confidence', self.fire_callback, 10)
        self.smoke_sub = self.create_subscription(Float32, '/smoke_level', self.smoke_callback, 10)

        self.alert_pub = self.create_publisher(Float32, '/fire_alert_level', 10)
        self.suppress_pub = self.create_publisher(Bool, '/activate_suppression', 10)

        self.fire_confidence = 0.0
        self.smoke_level = 0.0

        self.get_logger().info('Sensor Fusion Node started')

    def fire_callback(self, msg):
        self.fire_confidence = msg.data
        self.fuse()

    def smoke_callback(self, msg):
        self.smoke_level = msg.data
        self.fuse()

    def fuse(self):
        alert_level = (self.fire_confidence + self.smoke_level) / 2.0
        self.alert_pub.publish(Float32(data=alert_level))
        self.get_logger().info(f'Fused alert level: {alert_level:.2f}')

        if alert_level > 0.7:
            self.get_logger().warn('🔥 Fire detected! Triggering suppression.')
            self.suppress_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
