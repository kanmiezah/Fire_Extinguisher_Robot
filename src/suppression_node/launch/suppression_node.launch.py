from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='suppression_node',
            executable='suppression_node',
            name='suppression_node',
            output='screen',
            emulate_tty=True,  # Ensures proper log formatting in some terminals
            parameters=[  # Optional: can add dynamic runtime params here
                {'relay_pin': 17},
                {'timer_duration': 5.0}
            ],
            remappings=[
                # Example: ('/activate_suppression', '/custom_topic_name')
            ]
        )
    ])
