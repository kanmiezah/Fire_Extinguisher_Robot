from setuptools import setup
import os
from glob import glob

package_name = 'lidar_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel@todo.todo',
    description='Launch-only wrapper for RPLIDAR using rplidar_ros.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
