from setuptools import setup
import os
from glob import glob

package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Include Python package if __init__.py exists
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),  # Optional; skip if 'maps/' doesn't exist
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',  # Capitalize and use full name
    maintainer_email='your@email.com',
    description='Bringup launch file for full robot system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
