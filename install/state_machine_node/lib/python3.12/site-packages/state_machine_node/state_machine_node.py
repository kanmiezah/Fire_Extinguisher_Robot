import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_msgs.msg import String as StatusMsg
import time

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.cmd_pub = self.create_publisher(String, '/nav_command', 10)
        self.status_pub = self.create_publisher(StatusMsg, '/robot_state', 10)
        self.fire_sub = self.create_subscription(Bool, '/suppress_fire', self.fire_callback, 10)

        self.state = 'PATROL'
        self.fire_detected = False
        self.patrol_start_time = None
        self.patrol_duration = 10.0  # seconds of patrol

        self.timer = self.create_timer(1.0, self.run_state_machine)
        self.get_logger().info('State Machine Node with time-based patrol started')

    def fire_callback(self, msg):
        if msg.data:
            self.fire_detected = True

    def run_state_machine(self):
        self.get_logger().info(f'[STATE] {self.state}')
        self.status_pub.publish(StatusMsg(data=self.state))

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
            self.get_logger().info('Starting patrol')

        self.cmd_pub.publish(String(data='forward'))

        if now - self.patrol_start_time >= self.patrol_duration:
            self.get_logger().info('Finished timed patrol')
            self.patrol_start_time = None
            self.state = 'SCAN'

    def scan(self):
        self.get_logger().info('Scanning with AI...')
        if self.fire_detected:
            self.state = 'SUPPRESS'
        else:
            self.state = 'REST'

    def suppress(self):
        self.cmd_pub.publish(String(data='stop'))
        self.get_logger().warn('🔥 FIRE DETECTED — Executing suppression')
        self.fire_detected = False
        self.state = 'REST'

    def rest(self):
        self.get_logger().info('Resting...')
        time.sleep(3)
        self.state = 'PATROL'


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
