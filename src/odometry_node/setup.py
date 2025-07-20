from setuptools import setup
import os
from glob import glob

package_name = 'odometry_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Optional: if you add a launch file
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='your_email@example.com',
    description='ROS 2 node that reads encoder data over serial and publishes nav_msgs/Odometry for SLAM and navigation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher = odometry_node.odometry_publisher:main',
        ],
    },
)
