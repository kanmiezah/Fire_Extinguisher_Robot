from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Replace with the full path to your virtual environment's Python
    venv_python = os.path.join(os.getenv("HOME"), "fxb_ws/.venv/bin/python")

    return LaunchDescription([
        Node(
            package='smoke_sensor_node',
            executable='smoke_sensor_node',
            name='smoke_sensor_node',
            output='screen',
            prefix=f'env PYTHONPATH=$PYTHONPATH:{os.getenv("HOME")}/fxb_ws/ros2_ws/.venv/lib/python3.12/site-packages {venv_python}',  # Add PYTHONPATH and use venv's python
        ),
    ])
