from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Definition der Argumente (Zentrale Stelle für Messwerte)
    front_length_arg = DeclareLaunchArgument('frontLength', default_value='0.38')
    rear_length_arg = DeclareLaunchArgument('rearLength', default_value='0.38')
    wheel_diameter_arg = DeclareLaunchArgument('wheelDiameter', default_value='0.26') # Durchmesser in Meter
    axes_length_arg = DeclareLaunchArgument('axesLength', default_value='0.335')     # Spurweite/Achsabstand
    front_laser_arg = DeclareLaunchArgument('frontLaserLength', default_value='0.387')
    rear_laser_arg = DeclareLaunchArgument('rearLaserLength', default_value='-0.387')

    # 2. Knoten Definitionen
    
    base_node = Node(
        package='base',
        executable='base_node',
        name='base',
        output='screen',
        parameters=[{
            'wheelDiameter': LaunchConfiguration('wheelDiameter'),
            'axesLength': LaunchConfiguration('axesLength'),
            'frontLength': LaunchConfiguration('frontLength'),
            'rearLength': LaunchConfiguration('rearLength'),
            'use_sim_time': False
        }]
    )

    # Statische Transformationen (TF)
    axes_front_2_joint_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='AxesFront2JointFront',
        arguments=[LaunchConfiguration('frontLength'), '0', '0', '0', '0', '0', '1', 'axesFront', 'jointFront']
    )

    axes_rear_2_joint_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='AxesRear2JointRear',
        arguments=[LaunchConfiguration('rearLength'), '0', '0', '0', '0', '0', '1', 'jointRear', 'axesRear']
    )

    # Laser Transformationen
    laser_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='AxesFront2LaserFront',
        arguments=[LaunchConfiguration('frontLaserLength'), '0', '0', '0', '0', '0', '1', 'axesFront', 'laserFront']
    )

    laser_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='AxesRear2LaserRear',
        arguments=[LaunchConfiguration('rearLaserLength'), '0', '0', '0', '0', '0', '1', 'axesRear', 'laserRear']
    )

    # Basis Transformation
    base_link_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='BaseLink2AxesFront',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'axesFront']
    )

    # 3. Launch Description zusammenstellen
    return LaunchDescription([
        front_length_arg,
        rear_length_arg,
        wheel_diameter_arg,
        axes_length_arg,
        front_laser_arg,
        rear_laser_arg,
        base_node,
        axes_front_2_joint_front,
        axes_rear_2_joint_rear,
        laser_front,
        laser_rear,
        base_link_node
    ])