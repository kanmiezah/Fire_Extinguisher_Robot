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

        # Initialize SPI for MCP3008
        self.get_logger().info('Setting up SPI and MCP3008...')
        spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)
        cs = digitalio.DigitalInOut(board.D8)  # CE0 on Pi (Pin 24)
        self.mcp = MCP3008(spi, cs)
        self.chan = AnalogIn(self.mcp, MCP3008.P0)  # Read from CH0

        self.publisher_ = self.create_publisher(Float32, '/smoke_level', 10)
        self.timer = self.create_timer(1.0, self.publish_smoke_level)
        self.get_logger().info('Smoke sensor node has started.')

    def publish_smoke_level(self):
        voltage = self.read_sensor()
        msg = Float32()
        msg.data = voltage
        self.publisher_.publish(msg)
        self.get_logger().info(f'Smoke voltage: {voltage:.2f} V')

    def read_sensor(self):
        # Read real voltage from MQ-2 via MCP3008
        return round(self.chan.voltage, 2)


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
