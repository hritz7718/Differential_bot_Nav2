"""Start the laptop RGB-D detection process; run task_client separately."""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    return LaunchDescription([
        DeclareLaunchArgument('python', default_value=sys.executable),
        DeclareLaunchArgument('model', default_value='yolo11n.pt'),
        DeclareLaunchArgument('color_topic', default_value='/camera/camera/color/image_rect_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        ExecuteProcess(
            cmd=[LaunchConfiguration('python'), os.path.join(here, 'rgbd_detection.py'),
                 '--ros-args',
                 '-p', ['model:=', LaunchConfiguration('model')],
                 '-p', ['color_topic:=', LaunchConfiguration('color_topic')],
                 '-p', ['depth_topic:=', LaunchConfiguration('depth_topic')],
                 '-p', ['camera_info_topic:=', LaunchConfiguration('camera_info_topic')]],
            output='screen'),
    ])
