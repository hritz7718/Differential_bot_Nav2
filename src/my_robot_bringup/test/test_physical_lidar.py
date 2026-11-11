import importlib.util
from pathlib import Path
import shutil
import subprocess
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[2]
LAUNCH_FILE = ROOT / 'my_robot_bringup/launch/physical_bringup.launch.py'
XACRO_FILE = ROOT / 'my_robot_description/urdf/my_robot.urdf.xacro'


class PhysicalLidarTest(unittest.TestCase):
    def test_lidar_node_configuration(self):
        spec = importlib.util.spec_from_file_location('physical_bringup', LAUNCH_FILE)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with patch.object(module, 'get_package_share_directory', return_value='/tmp'), \
                patch.object(module, 'Node', return_value=None) as node, \
                patch.object(module, 'LaunchDescription', return_value=None), \
                patch.object(module, 'DeclareLaunchArgument', return_value=None) as argument:
            module.generate_launch_description()

        declarations = {call.args[0]: call.kwargs for call in argument.call_args_list}
        self.assertEqual(declarations['use_lidar']['default_value'], 'true')
        self.assertEqual(declarations['lidar_serial_port']['default_value'], '/dev/ttyUSB0')
        self.assertEqual(declarations['lidar_baud_rate']['default_value'], '115200')
        lidar = next(call.kwargs for call in node.call_args_list
                     if call.kwargs.get('package') == 'sllidar_ros2')
        self.assertEqual(lidar['executable'], 'sllidar_node')
        params = lidar['parameters'][0]
        self.assertEqual(params['frame_id'], 'lidar_link')
        self.assertEqual(params['channel_type'], 'serial')
        self.assertFalse(params['use_sim_time'])
        self.assertTrue(params['angle_compensate'])
        context = LaunchContext()
        context.launch_configurations['use_lidar'] = 'false'
        self.assertFalse(lidar['condition'].evaluate(context))
        context.launch_configurations['use_lidar'] = 'true'
        self.assertTrue(lidar['condition'].evaluate(context))

    @unittest.skipUnless(shutil.which('xacro'), 'xacro is not installed')
    def test_lidar_frame_and_gazebo_sensor(self):
        for use_control in ('false', 'true'):
            xml = subprocess.check_output(
                ['xacro', str(XACRO_FILE), f'use_ros2_control:={use_control}'],
                text=True)
            robot = ET.fromstring(xml)
            joints = [joint for joint in robot.findall('joint')
                      if joint.find('child').get('link') == 'lidar_link']
            self.assertEqual(len(joints), 1)
            self.assertEqual(joints[0].get('type'), 'fixed')
            self.assertEqual(joints[0].find('parent').get('link'), 'base_link')
            self.assertEqual(
                robot.find("gazebo[@reference='lidar_link']/sensor/plugin/frame_name").text,
                'lidar_link')


if __name__ == '__main__':
    unittest.main()
