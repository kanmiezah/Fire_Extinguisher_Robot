from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smoke_sensor_node',
            executable='smoke_sensor_node',
            name='smoke_sensor_node',
            output='screen',
            parameters=[{
                'analog_channel': 0,
                'read_interval': 1.0,
                'alert_threshold': 2.0,
            }]
        )
    ])
