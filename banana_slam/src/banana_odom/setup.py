from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'banana_odom'

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
        (os.path.join('share', package_name, 'config', 'slam_toolbox'), glob(os.path.join('config', 'slam_toolbox', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='EKF Odometry Fusion for FRA532 Lab 1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = banana_odom.ekf_node:main',
            'lidar_filter_node = banana_odom.lidar_filter_node:main',
            'trajectory_plotter = banana_odom.trajectory_plotter:main',
            'data_logger = banana_odom.data_logger:main',
            'plot_logs = banana_odom.plot_logs:main',
            'analyze_logs = banana_odom.analyze_logs:main',
            'sensor_noise_analyzer = banana_odom.sensor_noise_analyzer:main',
            'live_gaussian_viz = banana_odom.live_gaussian_viz:main',
        ],
    },
)
