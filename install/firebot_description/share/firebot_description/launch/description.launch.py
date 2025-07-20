from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Get path to Xacro file
    pkg_path = get_package_share_directory('firebot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # Use xacro to parse and generate robot description
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()

    # Robot state publisher node
    params = {'robot_description': robot_description_config}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
