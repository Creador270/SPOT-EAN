from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'old_spot_viz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            # Add this line to include all launch files in the launch directory
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikorose',
    maintainer_email='nikolay@mvnifest.com',
    description='A package to visualize the SPOT robot in Gazebo and ROS2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster_imu_node = old_spot_viz.tf_broadcater_imu:main'
        ],
    },
)
