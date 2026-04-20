import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    return LaunchDescription([

        DeclareLaunchArgument('plc_ip', default_value='192.168.0.43'),
        DeclareLaunchArgument('plc_port', default_value='50000'),
        DeclareLaunchArgument('xavier_ip', default_value='192.168.0.42'),
        DeclareLaunchArgument('xavier_port', default_value='50000'),

        #Die folgenden Communications Parameter sind nur für den im test ordner verfügbaren testserver
        #DeclareLaunchArgument('plc_ip', default_value='127.0.0.1'),
        #DeclareLaunchArgument('plc_port', default_value='50001'),  # Change to an available port
        #DeclareLaunchArgument('xavier_ip', default_value='127.0.0.1'),
        #DeclareLaunchArgument('xavier_port', default_value='50002'),  # Change to an available port

        DeclareLaunchArgument('plc_timeout', default_value='1.5'),
        DeclareLaunchArgument('zero_count_encoder', default_value='41229'),
        DeclareLaunchArgument('count_per_rotation_encoder', default_value='4096.0'),
        DeclareLaunchArgument('engine_acceleration', default_value='1000.0'),
        DeclareLaunchArgument('engine_jerk', default_value='10000.0'),
        DeclareLaunchArgument('period_Send_Read', default_value='0.05'),

        # Define the node
        Node(
            package='plc_connection',
            executable='plc_connection_node',
            name='PLC_Connection',
            output='screen',
            parameters=[{
                'PLC_IP': LaunchConfiguration('plc_ip'),
                'PLC_Port': LaunchConfiguration('plc_port'),
                'PLC_Timeout': LaunchConfiguration('plc_timeout'),
                'Xavier_Port': LaunchConfiguration('xavier_port'),
                'Xavier_IP': LaunchConfiguration('xavier_ip'),
                'ZeroCount_Encoder': LaunchConfiguration('zero_count_encoder'),
                'CountPerRotation_Encoder': LaunchConfiguration('count_per_rotation_encoder'),
                'Engine_Acceleration': LaunchConfiguration('engine_acceleration'),
                'Engine_Jerk': LaunchConfiguration('engine_jerk'),
                'Period_Send_Read': LaunchConfiguration('period_Send_Read'),
                'use_sim_time': False
            }],
            remappings=[
                # Uncomment and modify these lines if you need to remap topics
                # ('/engine/actualSpeed', '/engine/actualSpeed'),
                # ('/engine/targetSpeed', '/engine/targetSpeed'),
                # ('/engine/targetAcceleration', '/engine/targetAcceleration'),
                # ('/engine/targetTorque', '/engine/targetTorque'),
                # ('/sensors/bodyAngle', '/sensors/bodyAngle'),
            ]
        ),
    ])
