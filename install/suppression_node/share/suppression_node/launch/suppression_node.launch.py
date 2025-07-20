from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='suppression_node',
            executable='suppression_node',
            name='suppression_node',
            output='screen',
        )
    ])