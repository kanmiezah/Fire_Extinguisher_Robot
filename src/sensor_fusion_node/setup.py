from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensor_fusion_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Sensor fusion node for fire and smoke detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_fusion_node = sensor_fusion_node.sensor_fusion_node:main'
        ],
    },
)
