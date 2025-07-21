from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'motor_control_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial'  # Ensure this is included for serial communication
    ],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel@yourdomain.com',
    description='ROS 2 node to control motors over serial based on nav command input.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_node.motor_control_node:main'
        ],
    },
)
