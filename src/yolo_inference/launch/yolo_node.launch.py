from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare model_path argument
        DeclareLaunchArgument(
            'model_path',
            default_value='~/fxb_ws/FYP-AI-Model/best.pt',
            description='Path to the YOLOv8 .pt model'
        ),

        Node(
            package='yolo_inference',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path')
            }]
        )
    ])
