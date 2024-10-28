from setuptools import find_packages, setup

package_name = 'servo_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='creador270',
    maintainer_email='creadorjp@gmail.com',
    description='TODO: A driver package for the SPOT robot it subscribes to the joint_states  and aplies the commands to the motors.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver_node = servo_driver.servo_dirver:main'
        ],
    },
)
