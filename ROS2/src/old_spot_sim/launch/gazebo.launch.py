from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = True
    gui = True
    headless = False
    debug = False
    
    # Get the path to the URDF file
    urdf_dir = get_package_share_directory('old_spot_files')
    urdf = os.path.join(urdf_dir, 'URDF', 'spotmicroaiean.urdf')

    # Get the parent directory of the old_spot_files package
    parent_dir = os.path.dirname(urdf_dir)

    # Set the GAZEBO_MODEL_PATH
    os.environ["GAZEBO_MODEL_PATH"] = ":".join([os.path.join(urdf_dir, 'URDF'), os.path.join(parent_dir, 'meshes/stl_ean/STL'), parent_dir])    
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_spot_sim = get_package_share_directory('old_spot_sim')

    gazebo_command = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s',
             'libgazebo_ros_init.so', '-s', 
             'libgazebo_ros_factory.so'],
        output='screen'
    )
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-name', 'spot', '-file', urdf, 
                        #'-z', '0.1'
                        ],
                        output='screen')
    
    # Include the controller.launch.py file
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('old_spot_sim'), '/launch/controller.launch.py'])
    )
    return LaunchDescription([
        gazebo_command,
        spawn_entity,
        controller_launch,
    ])