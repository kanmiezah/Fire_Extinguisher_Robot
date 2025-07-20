from setuptools import setup
import os
from glob import glob

package_name = 'smoke_sensor_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'adafruit-circuitpython-mcp3xxx',
        'adafruit-blinka'
    ],
    zip_safe=True,
    maintainer='Nathaniel Miezah',
    maintainer_email='nathaniel@todo.todo',
    description='ROS 2 node that reads analog voltage from an MQ-2 smoke sensor via MCP3008 and publishes the result.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'smoke_sensor_node = smoke_sensor_node.smoke_sensor_node:main',
        ],
    },
)
