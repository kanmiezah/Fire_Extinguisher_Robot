from setuptools import find_packages, setup

package_name = 'camera_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',  # optional, if installing outside of ROS
    ],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel@yourdomain.com',
    description='Publishes frames from a camera as sensor_msgs/Image for YOLO and fire detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_node.camera_node:main'
        ],
    },
)
