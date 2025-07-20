import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import busio
import digitalio
import board
from adafruit_mcp3xxx.mcp3008 import MCP3008
from adafruit_mcp3xxx.analog_in import AnalogIn


class SmokeSensorNode(Node):
    def __init__(self):
        super().__init__('smoke_sensor_node')

        # Declare parameters with defaults
        self.declare_parameter('analog_channel', 0)
        self.declare_parameter('read_interval', 1.0)  # seconds
        self.declare_parameter('alert_threshold', 2.5)  # voltage in V

        channel_index = self.get_parameter('analog_channel').get_parameter_value().integer_value
        read_interval = self.get_parameter('read_interval').get_parameter_value().double_value
        self.alert_threshold = self.get_parameter('alert_threshold').get_parameter_value().double_value

        self.get_logger().info('🔧 Initializing MCP3008 on SPI...')

        try:
            spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
            cs = digitalio.DigitalInOut(board.D8)  # CE0 on Pi (Pin 24)
            self.mcp = MCP3008(spi, cs)
            self.chan = AnalogIn(self.mcp, getattr(MCP3008, f'P{channel_index}'))
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize MCP3008: {e}')
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(Float32, '/smoke_level', 10)
        self.timer = self.create_timer(read_interval, self.publish_smoke_level)

        self.get_logger().info(f'✅ SmokeSensorNode started (CH{channel_index}, interval={read_interval}s)')

    def publish_smoke_level(self):
        try:
            voltage = round(self.chan.voltage, 2)
        except Exception as e:
            self.get_logger().error(f'Failed to read sensor: {e}')
            return

        msg = Float32()
        msg.data = voltage
        self.publisher_.publish(msg)

        if voltage >= self.alert_threshold:
            self.get_logger().warn(f'🚨 High smoke level: {voltage:.2f} V')
        else:
            self.get_logger().debug(f'Smoke voltage: {voltage:.2f} V')

def main(args=None):
    rclpy.init(args=args)
    node = SmokeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
