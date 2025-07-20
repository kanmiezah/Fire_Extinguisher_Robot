from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='state_machine_node',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen',
            parameters=[],
            namespace=LaunchConfiguration('namespace', default=''),
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
