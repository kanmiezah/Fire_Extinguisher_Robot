from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'diff_drive_odometry_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name), ['README.md']),  # Optional: include README
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel.miezah@example.com',
    description='Node that reads wheel encoder data from Arduino and publishes nav_msgs/Odometry for SLAM/Nav2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_odometry_node = diff_drive_odometry_node.diff_drive_odometry_node:main'
        ],
    },
)
