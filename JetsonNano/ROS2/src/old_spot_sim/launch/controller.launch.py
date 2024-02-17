from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the URDF file
    urdf_dir = get_package_share_directory('old_spot_files')
    urdf = os.path.join(urdf_dir, 'URDF', 'spotmicroaiean.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    # Get the path to the mini_ros package
    mini_ros_dir = get_package_share_directory('old_spot_sim')

    # Load joint controller configurations from YAML file to parameter server
    config_file = os.path.join(mini_ros_dir, 'config', 'control.yaml')

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_position_controller', 
                   'joint_states_broadcaster'],
        output='screen',
        namespace='spot',
        parameters=[
            config_file],
    )
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        namespace='spot',
        # Remap the command topic to the correct topic
    remappings=[
        ('/joint_trajectory_command', 
         '/spot/joint_trajectory_position_controller/joint_trajectory')],
    )


    # Convert joint states to TF transforms for rviz, etc
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        namespace='spot',
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[config_file],
        namespace='spot',
        
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[config_file],
        namespace='spot',
    )

    return LaunchDescription([
        controller_manager,
        robot_state_publisher,
        controller_spawner,
        joint_state_publisher,
        joint_state_publisher_gui,
    ])