from setuptools import setup
import os
from glob import glob

package_name = 'fire_confidence_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Optional: include launch files if they exist
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nathaniel',
    maintainer_email='your@email.com',
    description='Node to calculate fire confidence based on /fire_targets',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_confidence_node = fire_confidence_node.fire_confidence_node:main',
        ],
    },
)
