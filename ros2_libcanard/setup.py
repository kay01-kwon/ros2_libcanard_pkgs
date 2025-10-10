import os
from glob import glob
from setuptools import setup

package_name = 'ros2_libcanard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))
    ]
)