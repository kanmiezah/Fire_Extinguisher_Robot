from setuptools import setup
import os
from glob import glob

package_name = 'yolo_inference'

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
        'ultralytics',       # ✅ ensure these are available
        'opencv-python',     # ✅ required for cv2
        'numpy',
    ],
    zip_safe=True,
    maintainer='Nathaniel Kangah Miezah',
    maintainer_email='nathaniel@example.com',
    description='YOLOv8 fire detection node using PyTorch (Ultralytics)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_inference.yolo_node:main',
        ],
    },
)
