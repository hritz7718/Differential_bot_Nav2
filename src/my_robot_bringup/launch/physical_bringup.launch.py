import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('my_robot_description')
    bringup_dir = get_package_share_directory('my_robot_bringup')

    xacro_file = os.path.join(
        description_dir, 'urdf', 'my_robot.urdf.xacro')
    controllers_file = os.path.join(
        bringup_dir, 'config', 'controllers.yaml')
    rviz_file = os.path.join(
        bringup_dir, 'rviz', 'urdf_config.rviz')

    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_baud_rate = LaunchConfiguration('lidar_baud_rate')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' serial_port:=', serial_port,
            ' baud_rate:=', baud_rate,
        ]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/serial/by-id/CHANGE_ME_ARDUINO_MEGA',
            description='Stable Arduino Mega serial device path'),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Arduino serial baud rate'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz'),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the physical RPLIDAR A1 driver'),
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLIDAR A1 serial device, separate from the Arduino'),
        DeclareLaunchArgument(
            'lidar_baud_rate',
            default_value='115200',
            description='RPLIDAR A1 serial baud rate'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False,
            }],
            output='screen'),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controllers_file],
            remappings=[
                ('~/robot_description', '/robot_description'),
            ],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--param-file', controllers_file,
            ],
            output='screen'),

        Node(
            package='my_robot_bringup',
            executable='twist_stamper.py',
            output='screen'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            condition=IfCondition(use_lidar),
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_serial_port,
                'serial_baudrate': ParameterValue(lidar_baud_rate, value_type=int),
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': True,
                'use_sim_time': False,
            }],
            output='screen'),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--param-file', controllers_file,
            ],
            output='screen'),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])
