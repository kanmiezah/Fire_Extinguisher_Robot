# motor_control_node/motor_control_node/motor_control_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

        self.subscription = self.create_subscription(
            String,
            '/nav_command',
            self.command_callback,
            10
        )

        self.get_logger().info('Motor Control Node Started')

    def command_callback(self, msg):
        command = msg.data.lower()
        if command in ['forward', 'backward', 'left', 'right', 'stop']:
            self.serial_port.write((command + '\n').encode())
            self.get_logger().info(f'Sent command: {command}')
        else:
            self.get_logger().warn(f'Unknown command: {command}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
