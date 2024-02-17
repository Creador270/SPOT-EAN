from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'old_spot_files'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include all STL files in the meshes directory
        ('share/' + package_name + '/meshes/stl_ean/STL', glob('meshes/stl_ean/STL/*.STL')),
        ('share/' + package_name + '/URDF', glob('URDF/*.urdf')),
        # Update the path to the model.config file
        ('share/' + package_name + '/URDF', ['URDF/model.config'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nikorose',
    maintainer_email='nikolay@mvnifest.com',
    description='A package to store the SPOT robot files.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
