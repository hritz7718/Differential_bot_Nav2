import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    hardware_arguments = [
        'serial_port', 'baud_rate', 'use_rviz', 'use_lidar',
        'lidar_serial_port', 'lidar_baud_rate',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the saved map YAML'),
        DeclareLaunchArgument(
            'params_file', default_value=params_file,
            description='Nav2 parameters YAML'),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/serial/by-id/CHANGE_ME_ARDUINO_MEGA',
            description='Stable Arduino Mega serial device path'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud_rate', default_value='115200'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'physical_bringup.launch.py')),
            launch_arguments={
                name: LaunchConfiguration(name) for name in hardware_arguments
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
            }.items()),
    ])
