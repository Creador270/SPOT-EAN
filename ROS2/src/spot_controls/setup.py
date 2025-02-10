from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'spot_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=['spot_controls', 'mini_ros', 'spotmicro']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gym'],
    zip_safe=True,
    maintainer='creador270',
    maintainer_email='creadorjp@gmail.com',
    description='A node who encapsule the logic of the pybulet robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movent_node = spot_controls.movent_node:main'
        ],
    },
)
