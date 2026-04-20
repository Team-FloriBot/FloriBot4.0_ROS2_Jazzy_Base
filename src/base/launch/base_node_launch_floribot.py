from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Argumente definieren
    return LaunchDescription([
        # Parameter base_node
        DeclareLaunchArgument('frontLength', default_value='-0.38'),
        DeclareLaunchArgument('rearLength', default_value='-0.38'),
        DeclareLaunchArgument('wheelDiameter', default_value='0.280'),
        DeclareLaunchArgument('axesLength', default_value='0.335'),
        
        # Parameter plc_connection
        DeclareLaunchArgument('plc_ip', default_value='192.168.0.43'),
        DeclareLaunchArgument('plc_port', default_value='50000'),
        DeclareLaunchArgument('xavier_ip', default_value='192.168.0.42'),
        DeclareLaunchArgument('xavier_port', default_value='50000'),
        DeclareLaunchArgument('plc_timeout', default_value='1.5'),
        DeclareLaunchArgument('zero_count_encoder', default_value='41229'),
        DeclareLaunchArgument('count_per_rotation_encoder', default_value='4096.0'),
        DeclareLaunchArgument('engine_acceleration', default_value='1000.0'),
        DeclareLaunchArgument('engine_jerk', default_value='10000.0'),
        DeclareLaunchArgument('period_Send_Read', default_value='0.05'),


        #base_node node launch
        Node(
            package='base',
            executable='base_node',
            name='base',
            output='screen',
            parameters=[
                {'wheelDiameter': -0.38},
                {'axesLength': -0.38}
            ]
        ),
        # tf2 static_transform_publisher Knoten
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='AxesFront2JointFront',
            output='screen',
            arguments=[LaunchConfiguration('frontLength'), '0', '0', '0', '0', '0', '1', 'axesFront', 'jointFront']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='AxesRear2JointRear',
            output='screen',
            arguments=[LaunchConfiguration('rearLength'), '0', '0', '0', '0', '0', '1', 'jointRear', 'axesRear']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='Front',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'axesFront']
        ),
        #plc_connection node launch
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

