import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share_directory = get_package_share_directory('cmd_vel_selector')

    parameter_file = os.path.join(
        package_share_directory,
        'config',
        'cmd_vel_selector.yaml'
    )

    cmd_vel_selector_node = Node(
        package='cmd_vel_selector',
        executable='cmd_vel_selector_node',
        name='cmd_vel_selector',
        output='screen',
        parameters=[parameter_file]
    )

    return LaunchDescription([
        cmd_vel_selector_node
    ])
