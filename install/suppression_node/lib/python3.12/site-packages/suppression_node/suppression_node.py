import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

# Optional hardware import (only runs on Raspberry Pi)
try:
    import RPi.GPIO as GPIO
    RPI_HARDWARE = True
except (ImportError, RuntimeError):
    RPI_HARDWARE = False
    print("⚠️ GPIO not available: running in simulation mode")


class SuppressionNode(Node):
    def __init__(self):
        super().__init__('suppression_node')

        self.relay_pin = 17  # BCM GPIO17
        self.timer_duration = 5.0
        self.deactivation_timer = None

        if RPI_HARDWARE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.relay_pin, GPIO.OUT)
            GPIO.output(self.relay_pin, GPIO.LOW)

        self.subscription = self.create_subscription(
            Bool,
            '/activate_suppression',
            self.callback,
            10
        )

        self.get_logger().info('Suppression node is ready.')

    def callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('🔥 Suppression activated.')
            if RPI_HARDWARE:
                GPIO.output(self.relay_pin, GPIO.HIGH)

            if self.deactivation_timer:
                self.deactivation_timer.cancel()
            self.deactivation_timer = self.create_timer(
                self.timer_duration,
                self.deactivate_relay_once
            )
        else:
            self.deactivate_relay()

    def deactivate_relay_once(self):
        self.get_logger().info(f'🕒 Auto deactivation after {self.timer_duration} seconds.')
        self.deactivate_relay()
        if self.deactivation_timer:
            self.deactivation_timer.cancel()
            self.deactivation_timer = None

    def deactivate_relay(self):
        if RPI_HARDWARE:
            GPIO.output(self.relay_pin, GPIO.LOW)
        self.get_logger().info('🛑 Suppression deactivated.')

    def destroy_node(self):
        if RPI_HARDWARE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SuppressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()