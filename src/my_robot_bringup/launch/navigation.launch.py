import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('my_robot_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    default_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='NAv2 map file to load'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo time for simulation; set false on hardware'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'slam': 'False',
            }.items()),
    ])
