from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
import xacro

def generate_launch_description():
    package_name = "robot_description"
    
    # Sim-Time Argument (Standard auf False für den echten Roboter)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        "Floribot.urdf.xacro"
    )
    
    doc = xacro.process_file(urdf_path)
    robot_description_content = doc.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time
            }]
        )
    ])
