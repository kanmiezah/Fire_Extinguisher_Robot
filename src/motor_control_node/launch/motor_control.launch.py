from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port connected to Arduino motor controller'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='9600',
            description='Baud rate for serial communication'
        ),
        Node(
            package='motor_control_node',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }]
        )
    ])
