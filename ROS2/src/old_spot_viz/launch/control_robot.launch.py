import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    share_dir = get_package_share_directory('mpu6050driver')
    parameter_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'mpu6050.yaml'),
                                           description='Path to the ROS2 parameters file to use.')
        # Launch mpu6050 driver
        Node(
            package='mpu6050driver',
            executable='mpu6050driver',
            name='mpu6050driver_node',
            output="screen",
            emulate_tty=True,
            parameters=[parameter_file]
        ),
        # Launch imu complementary filter
        Node(
            package='imu_complementary_filter',
            executable='imu_complementary_filter_node',
            output='screen',
            ),
        # Launch servo driver node
        Node(
            package='servo_driver',
            executable='servo_driver_node',
            output='screen',
            # parameters=[{'offset_joints': debug_mode}]
        ),
        # Launch the joint_state_publisher node
        Node(
            package='spot_controls',
            executable='movent_node',
            output='screen',
            parameters=[{'fr': 30.0}],
            ),
        #joystick Node
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
            output='screen',
        )
    ])
