import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to the URDF file
    urdf_dir = get_package_share_directory('old_spot_files')
    urdf = os.path.join(urdf_dir, 'URDF', 'spotmicroaiean.urdf')

    # Get the path to the RViz config file
    config_dir = get_package_share_directory('old_spot_viz')
    rviz_config = os.path.join(config_dir, 'config', 'config.rviz')
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    debug_mode = LaunchConfiguration('debug_mode')
        
    return LaunchDescription([

        launch.actions.DeclareLaunchArgument('debug_mode', default_value='false', description='Activar modo debug para motores'),

        # Declare the launch argument
        DeclareLaunchArgument(
            'urdf',
            default_value=urdf,
            description='path to urdf file'
        ),

        # Launch the robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Launch servo driver node
        Node(
            package='servo_driver',
            executable='servo_driver_node',
            output='screen',
            parameters=[{'offset_joints': debug_mode}]
        ),

        # Launch broadcaster IMU node
        Node(
            package='old_spot_viz',
            executable='tf_broadcaster_imu_node',
            output='screen'
        ),

        # Launch the rviz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # Launch the joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            condition=launch.conditions.IfCondition(debug_mode)
        ),

        # Launch the joint_state_publisher node
        Node(
            package='inv_k',
            executable='inv_k_node',
            name='joint_state_publisher',
            output='screen',
            # parameters=[{'robot_description': robot_desc}],
            condition=launch.conditions.UnlessCondition(debug_mode) 
        ),
    ])