from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_node',
            executable='odometry_publisher',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'baudrate': 115200
            }],
            output='screen'
        )
    ])
