from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='old_spot_sim',
            executable='spot_sm',
            name='spot_sm',
            output='screen',
            parameters=[{'frequency': 200.0}],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='spot_joy',
            respawn=True,
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.05},
            ],
        ),
        Node(
            package='old_spot_sim',
            executable='teleop_node',
            name='spot_teleop',
            output='screen',
            parameters=[
                {'frequency': 200.0},
                {'axis_linear_x': 4},
                {'axis_linear_y': 3},
                {'axis_linear_z': 1},
                {'axis_angular': 0},
                {'scale_linear': 1.0},
                {'scale_angular': 1.0},
                {'button_switch': 0},
                {'button_estop': 1},
            ],
        ),
        Node(
            package='old_spot_sim',
            executable='spot_pybullet_interface',
            name='spot_pybullet',
            output='screen',
        ),
    ])