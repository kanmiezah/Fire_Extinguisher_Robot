from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Camera node
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),

        # YOLO Inference node
        Node(
            package='yolo_inference',
            executable='yolo_node',
            name='yolo_node',
            output='screen'
        ),

        # Fire confidence node
        Node(
            package='fire_confidence_node',
            executable='fire_confidence_node',
            name='fire_confidence_node',
            output='screen'
        ),

        # Smoke sensor node
        Node(
            package='smoke_sensor_node',
            executable='smoke_sensor_node',
            name='smoke_sensor_node',
            output='screen'
        ),

        # Fire suppression node
        Node(
            package='suppression_node',
            executable='suppression_node',
            name='suppression_node',
            output='screen'
        ),

        # State machine node
        Node(
            package='state_machine_node',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen'
        ),

        # Sensor fusion node
        Node(
            package='sensor_fusion_node',
            executable='sensor_fusion_node',
            name='sensor_fusion_node',
            output='screen'
        ),

        # Motor control node
        Node(
            package='motor_control_node',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen'
        ),

        # RPLIDAR node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 460800,
                'frame_id': 'laser_frame',
                'angle_compensate': True
            }]
        ),

        # Diff drive odometry node
        Node(
            package='diff_drive_odometry_node',
            executable='diff_drive_odometry_node',
            name='odometry_node',
            output='screen'
        ),

        # Nav2 bringup (navigation stack)
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
                       ])]
        ),

        # Robot State Publisher with URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('firebot_description'),
                        'urdf',
                        'firebot.urdf'
                    ])
                ])
            }]
        ),
    ])
