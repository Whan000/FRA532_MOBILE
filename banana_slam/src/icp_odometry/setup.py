from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'icp_odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ICP Odometry Refinement for FRA532 Lab 1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_node = icp_odometry.icp_node:main',
            'simple_icp_node = icp_odometry.simple_icp_node:main',
            'lidar_filter_node = icp_odometry.lidar_filter_node:main',
        ],
    },
)
