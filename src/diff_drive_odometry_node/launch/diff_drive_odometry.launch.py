from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diff_drive_odometry_node',
            executable='diff_drive_odometry_node',
            name='diff_drive_odometry_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB2'},
                {'baudrate': 9600},
                {'frame_id': 'odom'},
                {'child_frame_id': 'base_link'}
            ]
        )
    ])
