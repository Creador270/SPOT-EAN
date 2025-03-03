import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    share_dir = get_package_share_directory('mpu6050driver')
    launch_imu_file = default_value=os.path.join(share_dir, 'launch', 'mpu6050driver_launch.py')

    return LaunchDescription([

        # DeclareLaunchArgument('params_file', default_value=os.path.join(
        #                                         share_dir, 'params', 'mpu6050.yaml'),
        #                                         description='Path to the ROS2 parameters file to use.'),
        #  Launch mpu6050 driver
        # Node(
        #     package='mpu6050driver',
        #     executable='mpu6050driver',
        # ),
        # # Launch imu complementary filter
        # Node(
        #     package='imu_complementary_filter',
        #     executable='imu_complementary_filter_node',
        #     output='screen',
        #     ),

        # Call launch file IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_imu_file)
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
            parameters=[{'fr': 60.0}],
            ),
        #joystick Node
        Node(
            package='joystick_ros2',
            executable='joystick_ros2',
            output='screen',
        )
    ])
