from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fire_confidence_node',
            executable='fire_confidence_node',
            name='fire_confidence_node',
            output='screen',
            parameters=[{
                'max_detections': 5  # You can override this from CLI or config YAML
            }]
        )
    ])
