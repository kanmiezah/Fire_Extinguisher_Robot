from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'suppression_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Supports nested structure like suppression_node/suppression_node
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Miezah',
    maintainer_email='nathaniel@todo.todo',
    description='ROS 2 node to control a relay-based fire suppression system via GPIO.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'suppression_node = suppression_node.suppression_node:main',
        ],
    },
)
