import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 9600)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to serial port: {port} at {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serial_port = None

        self.allowed_commands = ['forward', 'backward', 'left', 'right', 'stop']

        self.subscription = self.create_subscription(
            String,
            '/nav_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg):
        command = msg.data.lower()
        if command in self.allowed_commands:
            if self.serial_port:
                self.serial_port.write((command + '\n').encode())
                self.get_logger().info(f'Sent command: {command}')
            else:
                self.get_logger().error('Serial port is not initialized.')
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
            self.get_logger().info("Closed serial port.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
