from setuptools import find_packages
from setuptools import setup

setup(
    name='system_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('system_msgs', 'system_msgs.*')),
)
