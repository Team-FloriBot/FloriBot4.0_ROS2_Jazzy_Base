from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params_dir = os.path.join(get_package_share_directory('base'), 'params')
    hardware_params = os.path.join(params_dir, 'hardware_params.yaml')
    kinematics_params = os.path.join(params_dir, 'kinematics_params.yaml')

    return LaunchDescription([
        Node(
            package='base',
            executable='hardware_node',
            name='hardware_node',
            output='screen',
            parameters=[hardware_params],
        ),
        Node(
            package='base',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[kinematics_params],
        ),
    ])
