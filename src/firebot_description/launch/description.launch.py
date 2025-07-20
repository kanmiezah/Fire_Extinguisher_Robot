from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_path = get_package_share_directory('firebot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node
    ])
