from pathlib import Path
import math
import subprocess
import xml.etree.ElementTree as ET


XACRO = Path(__file__).resolve().parents[1] / 'urdf' / 'my_robot.urdf.xacro'


def robot_description(hardware=False):
    xml = subprocess.check_output(
        ['xacro', str(XACRO), f'use_ros2_control:={str(hardware).lower()}'], text=True)
    return ET.fromstring(xml)


def vector(text):
    return tuple(float(value) for value in text.split())


def close_vector(actual, expected, tolerance=1e-9):
    return all(abs(a - b) < tolerance for a, b in zip(actual, expected))


def test_frames_geometry_and_hardware():
    robot = robot_description(hardware=True)
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    assert close_vector(vector(joints['base_joint'].find('origin').get('xyz')), (0, 0, 0.034))
    expected = {
        'base_left_wheel_joint': (0, 0.1135, 0),
        'base_right_wheel_joint': (0, -0.1135, 0),
        'base_caster_wheel_joint': (0.095, 0, -0.019),
        'base_middle_plate_joint': (0.050, 0, 0.076),
        'base_upper_plate_joint': (0.050, 0, 0.126),
        'base_left_motor_housing_joint': (0, 0.035, -0.001),
        'base_right_motor_housing_joint': (0, -0.035, -0.001),
        'base_lidar_joint': (0.028, 0, 0.196),
        'base_camera_joint': (0.095, 0, 0.151),
    }
    for name, position in expected.items():
        assert close_vector(vector(joints[name].find('origin').get('xyz')), position), name
    for name in ('base_left_wheel_joint', 'base_right_wheel_joint'):
        assert joints[name].get('type') == 'continuous'
        assert close_vector(vector(joints[name].find('axis').get('xyz')), (0, 1, 0))
    assert close_vector(vector(joints['base_lidar_joint'].find('origin').get('rpy')),
                        (0, 0, -math.pi / 2))
    controlled = {joint.get('name') for joint in robot.findall('./ros2_control/joint')}
    assert controlled == {'base_left_wheel_joint', 'base_right_wheel_joint'}


def test_total_mass_and_centre_of_gravity():
    robot = robot_description()
    joints = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    total_mass = 0.0
    moment = [0.0, 0.0, 0.0]
    base_height = vector(joints['base_link'].find('origin').get('xyz'))[2]
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass = float(inertial.find('mass').get('value'))
        assert mass > 0
        inertia = inertial.find('inertia')
        assert all(float(inertia.get(axis)) > 0 for axis in ('ixx', 'iyy', 'izz'))
        local = vector(inertial.find('origin').get('xyz'))
        if link.get('name') == 'base_link':
            position = (local[0], local[1], local[2] + base_height)
        else:
            joint = joints[link.get('name')]
            assert joint.find('parent').get('link') == 'base_link'
            offset = vector(joint.find('origin').get('xyz'))
            # All child inertial origins are zero, including the rotated sensor and wheels.
            assert close_vector(local, (0, 0, 0))
            position = (offset[0], offset[1], offset[2] + base_height)
        total_mass += mass
        for index in range(3):
            moment[index] += mass * position[index]
    assert abs(total_mass - 1.350) < 1e-9
    assert close_vector(tuple(value / total_mass for value in moment),
                        (0.03655185185, 0, 0.11497037037), tolerance=1e-8)


def test_simulated_lidar_and_drive_remain_available():
    robot = robot_description()
    assert robot.find('./ros2_control') is None
    assert robot.find("gazebo[@reference='lidar_link']/sensor/plugin/frame_name").text == 'lidar_link'
    plugin = robot.find("gazebo/plugin[@name='diff_drive_controller']")
    assert plugin is not None
    assert abs(float(plugin.find('wheel_separation').text) - 0.227) < 1e-9
    assert abs(float(plugin.find('wheel_diameter').text) - 0.068) < 1e-9
