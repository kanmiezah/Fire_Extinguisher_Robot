from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'state_machine_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Ensures all Python submodules are included
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel@example.com',
    description='State machine controlling patrol, scan, suppress, and rest logic in a firefighting robot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_machine_node = state_machine_node.state_machine_node:main',
        ],
    },
)
