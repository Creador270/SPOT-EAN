from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'old_spot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'old_spot_sim',
        'old_spot_sim.ars_lib',
        'old_spot_sim.mini_bullet',
    ],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include all launch files in the launch directory
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    # install_requires=['setuptools'],
    zip_safe=True,
    author='Nikolay Prieto PhD',
    author_email='enprietop@unal.edu.co',
    maintainer='Nikolay Prieto',
    maintainer_email='enprietop@unal.edu.co',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
    ],
    description=(
        'ROS2 package for old_spot_sim.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'mini_bullet = old_spot_sim.mini_bullet:main',
            # 'ars_lib = old_spot_sim.ars_lib:main',
        ],
    },
)