import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time

class StateMachineNode(Node):
    """
    A simple timed finite state machine controlling robot behavior:
    PATROL -> SCAN -> SUPPRESS (if fire) -> REST -> repeat.
    """

    def __init__(self):
        super().__init__('state_machine_node')

        self.cmd_pub = self.create_publisher(String, '/nav_command', 10)
        self.status_pub = self.create_publisher(String, '/robot_state', 10)
        self.fire_sub = self.create_subscription(Bool, '/suppress_fire', self.fire_callback, 10)

        self.state = 'PATROL'
        self.fire_detected = False

        self.patrol_start_time = None
        self.rest_start_time = None

        self.patrol_duration = 10.0  # seconds
        self.rest_duration = 3.0     # seconds

        self.timer = self.create_timer(1.0, self.run_state_machine)

        self.get_logger().info('State Machine Node with timer-based state transitions started')

    def fire_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("🔥 Fire detection signal received.")
            self.fire_detected = True

    def run_state_machine(self):
        self.get_logger().info(f'[STATE] {self.state}')
        self.status_pub.publish(String(data=self.state))

        if self.state == 'PATROL':
            self.patrol()
        elif self.state == 'SCAN':
            self.scan()
        elif self.state == 'SUPPRESS':
            self.suppress()
        elif self.state == 'REST':
            self.rest()

    def patrol(self):
        now = time.time()

        if self.patrol_start_time is None:
            self.patrol_start_time = now
            self.get_logger().info('🚶 Starting patrol...')

        self.cmd_pub.publish(String(data='forward'))

        if now - self.patrol_start_time >= self.patrol_duration:
            self.get_logger().info('✅ Finished patrol.')
            self.patrol_start_time = None
            self.state = 'SCAN'

    def scan(self):
        self.get_logger().info('🔍 Scanning for fire...')
        if self.fire_detected:
            self.state = 'SUPPRESS'
        else:
            self.state = 'REST'

    def suppress(self):
        self.cmd_pub.publish(String(data='stop'))
        self.get_logger().warn('🔥 FIRE DETECTED — Executing suppression...')
        self.fire_detected = False
        self.state = 'REST'

    def rest(self):
        now = time.time()

        if self.rest_start_time is None:
            self.rest_start_time = now
            self.get_logger().info('😴 Resting...')

        if now - self.rest_start_time >= self.rest_duration:
            self.rest_start_time = None
            self.get_logger().info('🟢 Restarting patrol...')
            self.state = 'PATROL'

    def destroy_node(self):
        self.get_logger().info('🛑 State machine node shutting down...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
