import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    base_share_directory = get_package_share_directory('base')
    selector_share_directory = get_package_share_directory('cmd_vel_selector')

    base_launch_file = os.path.join(
        base_share_directory,
        'launch',
        'base_node.launch.py'
    )

    selector_parameter_file = os.path.join(
        selector_share_directory,
        'config',
        'cmd_vel_selector.yaml'
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_file)
    )

    cmd_vel_selector_node = Node(
        package='cmd_vel_selector',
        executable='cmd_vel_selector_node',
        name='cmd_vel_selector',
        output='screen',
        parameters=[selector_parameter_file]
    )

    return LaunchDescription([
        cmd_vel_selector_node,
        base_launch
    ])
