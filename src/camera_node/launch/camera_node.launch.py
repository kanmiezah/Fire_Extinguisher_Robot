from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch-time arguments with default values
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('frame_width', default_value='640'),
        DeclareLaunchArgument('frame_height', default_value='480'),

        # Node with parameters passed in
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'camera_index': LaunchConfiguration('camera_index')},
                {'frame_width': LaunchConfiguration('frame_width')},
                {'frame_height': LaunchConfiguration('frame_height')},
            ]
        )
    ])
