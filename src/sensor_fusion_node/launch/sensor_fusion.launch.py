from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_fusion_node',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen'
            parameters=[{'alert_threshold': 0.7}],
            remappings=[('/some_input', '/actual_topic')]

        )
    ])
