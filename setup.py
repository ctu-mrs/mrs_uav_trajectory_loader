import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mrs_multi_uav_trajectory_loader'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch/*.launch.py'))
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config/*.yaml'))
        ),
        (
            os.path.join('share', package_name, 'config/trajectory'),
            glob('config/trajectory/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tobias Vinklarek',
    maintainer_email='vinkltob@fel.cvut.cz',
    description='Minimal ROS2 multi-uav trajectory loader.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_loader_node = trajectory_loader.trajectory_loader:main',
        ],
    },
)
