from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0',
            description='Serial port for RPLIDAR'
        ),
        DeclareLaunchArgument(
            'serial_baudrate', default_value='115200',
            description='Baudrate for RPLIDAR'
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baudrate': LaunchConfiguration('serial_baudrate')},
                {'frame_id': 'laser_frame'},
                {'scan_mode': 'Standard'},
                {'angle_compensate': True},
                {'inverted': False},
                {'auto_standby': True},
            ]
        )
    ])
