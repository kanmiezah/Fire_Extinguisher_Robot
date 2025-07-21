from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('bringup'),
                'config',
                'slam_config.yaml'
            ])]
        ),

        # Nav2 bringup
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            arguments=['--ros-args', '--params-file',
                PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'config',
                    'nav2_params.yaml'
                ])
            ]
        )
    ])
