import ast
import importlib.util
from pathlib import Path
import re
import time
import uuid
from unittest.mock import patch

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml


ROOT = Path(__file__).resolve().parents[2]
BRINGUP = ROOT / 'my_robot_bringup'
HARDWARE_LAUNCH = BRINGUP / 'launch' / 'physical_bringup.launch.py'
NAVIGATION_LAUNCH = BRINGUP / 'launch' / 'physical_navigation.launch.py'
PHYSICAL_CONFIG = BRINGUP / 'config' / 'nav2_params_physical.yaml'
CONTROLLERS = BRINGUP / 'config' / 'controllers.yaml'
FIRMWARE = ROOT.parent / 'firmware' / 'diffbot_mega' / 'diffbot_mega.ino'
STAMPER = BRINGUP / 'scripts' / 'twist_stamper.py'


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_navigation_requires_monitor_output_for_hardware():
    module = load_launch(NAVIGATION_LAUNCH)
    with patch.object(module, 'get_package_share_directory', return_value='/robot_bringup'), \
            patch.object(module, 'IncludeLaunchDescription', return_value=None) as include, \
            patch.object(module, 'PythonLaunchDescriptionSource', side_effect=lambda value: value), \
            patch.object(module, 'LaunchDescription', return_value=None), \
            patch.object(module, 'DeclareLaunchArgument', return_value=None) as declare:
        module.generate_launch_description()

    declarations = {call.args[0]: call.kwargs for call in declare.call_args_list}
    assert declarations['params_file']['default_value'].endswith('nav2_params_physical.yaml')
    hardware = next(call for call in include.call_args_list
                    if call.args[0].endswith('physical_bringup.launch.py'))
    assert dict(hardware.kwargs['launch_arguments'])['command_input_topic'] == '/cmd_vel_safe'
    navigation = next(call for call in include.call_args_list
                      if call.args[0].endswith('/navigation.launch.py'))
    assert dict(navigation.kwargs['launch_arguments'])['use_sim_time'] == 'false'


def test_manual_bringup_keeps_teleop_default():
    module = load_launch(HARDWARE_LAUNCH)
    with patch.object(module, 'get_package_share_directory', return_value='/robot_bringup'), \
            patch.object(module, 'Node', return_value=None) as node, \
            patch.object(module, 'LaunchDescription', return_value=None), \
            patch.object(module, 'DeclareLaunchArgument', return_value=None) as declare:
        module.generate_launch_description()
    declarations = {call.args[0]: call.kwargs for call in declare.call_args_list}
    assert declarations['command_input_topic']['default_value'] == '/cmd_vel'
    bridge = next(call.kwargs for call in node.call_args_list
                  if call.kwargs.get('executable') == 'twist_stamper.py')
    assert bridge['remappings'][0][0] == '/cmd_vel'


def test_physical_config_matches_chassis_and_command_path():
    with PHYSICAL_CONFIG.open() as source:
        config = yaml.safe_load(source)
    with CONTROLLERS.open() as source:
        drive = yaml.safe_load(source)['diff_drive_controller']['ros__parameters']

    for section in ('bt_navigator', 'controller_server', 'velocity_smoother'):
        assert config[section]['ros__parameters']['odom_topic'] == '/diff_drive_controller/odom'
    for name in ('local_costmap', 'global_costmap'):
        costmap = config[name][name]['ros__parameters']
        assert costmap['global_frame'] == ('odom' if name == 'local_costmap' else 'map')
        footprint = ast.literal_eval(costmap['footprint'])
        assert min(p[0] for p in footprint) <= -0.034
        assert max(p[0] for p in footprint) >= 0.110
        assert min(p[1] for p in footprint) <= -0.126
        assert max(p[1] for p in footprint) >= 0.126
        assert costmap['footprint_padding'] > 0
    monitor = config['collision_monitor']['ros__parameters']
    assert monitor['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert monitor['cmd_vel_out_topic'] == '/cmd_vel_safe'
    assert monitor['source_timeout'] > 0
    assert monitor['scan']['topic'] == '/scan'
    assert monitor['StopZone']['action_type'] == 'stop'
    assert not config['planner_server']['ros__parameters']['GridBased']['allow_unknown']

    planner = config['controller_server']['ros__parameters']['FollowPath']
    smoother = config['velocity_smoother']['ros__parameters']
    assert planner['max_vel_x'] <= drive['linear.x.max_velocity']
    assert planner['max_vel_theta'] <= drive['angular.z.max_velocity']
    assert smoother['max_velocity'][0] == planner['max_vel_x']
    assert smoother['max_velocity'][2] == planner['max_vel_theta']
    max_wheel = (planner['max_vel_x'] + planner['max_vel_theta'] *
                 drive['wheel_separation'] / 2.0) / drive['wheel_radius']
    firmware = FIRMWARE.read_text()
    max_firmware = float(re.search(r'kMaximumWheelSpeed\s*=\s*([0-9.]+)', firmware).group(1))
    assert max_wheel < max_firmware


def test_physical_profile_uses_real_time():
    def walk(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == 'use_sim_time':
                    assert item is False
                walk(item)
        elif isinstance(value, list):
            for item in value:
                walk(item)

    with PHYSICAL_CONFIG.open() as source:
        walk(yaml.safe_load(source))


def test_bridge_accepts_only_remapped_input():
    prefix = '/diffbot_test_' + uuid.uuid4().hex
    safe_topic = prefix + '/safe'
    output_topic = prefix + '/output'
    rclpy.init(args=['--ros-args', '-r', '/cmd_vel:=' + safe_topic,
                     '-r', '/diff_drive_controller/cmd_vel:=' + output_topic])
    bridge = None
    node = None
    executor = SingleThreadedExecutor()
    try:
        bridge = load_launch(STAMPER).TwistStamper()
        node = rclpy.create_node('test_diffbot_bridge', use_global_arguments=False)
        received = []
        node.create_subscription(TwistStamped, output_topic, received.append, 10)
        safe = node.create_publisher(Twist, safe_topic, 10)
        other = node.create_publisher(Twist, prefix + '/other', 10)
        executor.add_node(bridge)
        executor.add_node(node)
        deadline = time.monotonic() + 5.0
        while (safe.get_subscription_count() < 1 or
               node.count_publishers(output_topic) < 1) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert safe.get_subscription_count() == 1
        assert node.count_publishers(output_topic) == 1
        command = Twist()
        command.linear.x = 0.05
        other.publish(command)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert not received
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            safe.publish(command)
            executor.spin_once(timeout_sec=0.05)
        assert received and received[-1].twist.linear.x == 0.05
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if bridge is not None:
            bridge.destroy_node()
        rclpy.shutdown()
