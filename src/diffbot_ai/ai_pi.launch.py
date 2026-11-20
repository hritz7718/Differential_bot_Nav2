"""Start Pi-side camera and preview executor, optionally the existing Nav2 stack."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    camera_launch = os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
    navigation_launch = os.path.join(get_package_share_directory('my_robot_bringup'), 'launch', 'physical_navigation.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('start_navigation', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/CHANGE_ME_ARDUINO_MEGA'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            launch_arguments={
                'enable_color': 'true', 'enable_depth': 'true',
                'align_depth.enable': 'true', 'enable_sync': 'true',
                'publish_tf': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            condition=IfCondition(LaunchConfiguration('start_navigation')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'serial_port': LaunchConfiguration('serial_port'),
                'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
                'use_lidar': 'true', 'use_rviz': 'false',
            }.items()),
        ExecuteProcess(
            cmd=[sys.executable, os.path.join(here, 'task_executor.py'),
                 '--ros-args', '-p', ['allow_motion:=', LaunchConfiguration('allow_motion')]],
            output='screen'),
    ])
