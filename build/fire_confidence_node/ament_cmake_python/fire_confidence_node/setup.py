from setuptools import find_packages
from setuptools import setup

setup(
    name='fire_confidence_node',
    version='0.1.0',
    packages=find_packages(
        include=('fire_confidence_node', 'fire_confidence_node.*')),
)
