# Custom DiffBot: ROS 2 Navigation

A custom differential-drive robot with a Gazebo Harmonic simulation and a
physical ROS 2 control stack. The physical robot uses a Raspberry Pi 5, Arduino
Mega, wheel encoders, L298N driver, and RPLIDAR A1. Its autonomous-navigation
configuration is a supervised starting point and has not been safety validated.

## Features

- Two-wheel differential drive with a caster wheel
- Simulated 360° LiDAR publishing `/scan`
- Simulated camera
- Odometry and TF published by `diff_drive_controller`
- AMCL localization and Nav2 navigation using a saved map
- Custom robot footprint and local/global costmap configuration
- Physical `ros2_control` hardware interface and RPLIDAR driver launch
- Separate Jazzy physical navigation profile with a collision-monitor command path

## Packages

| Package | Contents |
| --- | --- |
| `my_robot_description` | Xacro model, sensors, and robot visualization |
| `my_robot_bringup` | Gazebo world, launch files, RViz configuration, and Nav2 parameters |
| `my_robot_hardware` | Serial hardware interface for the Arduino Mega |

The TF chain used for navigation is:

map → odom → base_footprint → base_link → lidar_link


AMCL or SLAM Toolbox provides `map → odom`; the drive controller provides
`odom → base_footprint` in simulation and on the physical robot.

## Requirements

The simulation targets Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Install
ROS 2 Jazzy, then install the simulation and navigation packages:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-rviz2 \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

The physical RPLIDAR driver is a separate dependency. The `src/sllidar_ros2`
entry in this checkout is an empty gitlink without a configured `.gitmodules`;
obtain the driver separately if physical LiDAR bringup is required. See the
physical bringup guide for hardware requirements.

## Build the simulation

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotics/Differential_bot_Nav2
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robotics/Differential_bot_Nav2
source install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

The launch starts Gazebo with the existing world, spawns DiffBot, activates the
joint state and differential drive controllers, and starts RViz. To run without
both graphical applications:

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml gui:=false use_rviz:=false
```

`gui` controls Gazebo's client; `use_rviz` independently controls RViz. The
headless server still needs an Ogre 2 rendering backend for LiDAR and camera.
The simulation uses `/clock`, `/cmd_vel` (`geometry_msgs/msg/Twist`), `/odom`,
`/joint_states`, `/scan`, `/camera/image_raw`, and `/camera/camera_info`.
The existing command adapter stamps `/cmd_vel` for Jazzy's controller. For a
brief manual motion check in another sourced terminal:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop publishing with Ctrl-C; the controller stops after its command timeout.
The world references an unavailable Classic bowl mesh, so the bowl visual uses
its existing cylindrical collision dimensions instead.

Run only one simulation on a ROS domain and Gazebo Transport partition at a
time. For concurrent simulations, set a distinct `ROS_DOMAIN_ID` and
`GZ_PARTITION` in every terminal used for each simulation, including its Nav2
and SLAM terminals.

## Create a map

With the simulation running, start SLAM Toolbox in another terminal:

```bash
source ~/robotics/Differential_bot_Nav2/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Drive the robot through the world, then save the map:

```bash
mkdir -p ~/my_maps
ros2 run nav2_map_server map_saver_cli -f ~/my_maps/my_world \
  --ros-args -p map_subscribe_transient_local:=true
```

This produces `my_world.yaml` and a map image. Stop SLAM Toolbox before starting saved-map navigation so it does not compete with AMCL for `map → odom`.

## Navigate with a saved map

Keep the simulation running and launch Nav2 in a separate terminal:

```bash
source ~/robotics/Differential_bot_Nav2/install/setup.bash
ros2 launch my_robot_bringup navigation.launch.py \
  map:=$HOME/my_maps/my_world.yaml
```

In RViz, set **Fixed Frame** to `map`, use **2D Pose Estimate** to set the robot's starting position and heading, then send a **2D Goal Pose**.

A saved map is required for this launch and is not included in the repository. The map must match the Gazebo world: mismatched walls or an incorrect initial pose can cause AMCL pose jumps and misaligned costmaps.

In the Extras folder, I have saved a template file to manipulate the robot through without using RViz 2D Goal pose buttons, u can run the python files with your coordinates and it will run automatically.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Wheel separation | 0.227 m |
| Wheel diameter | 0.068 m |
| Odometry update rate | 50 Hz |
| LiDAR update rate | 10 Hz |
| Simulated LiDAR range | 0.15–12 m |
| LiDAR samples | 360 per scan |

## Physical Robot

See [physical bringup](docs/PHYSICAL_BRINGUP.md) for wiring, firmware and
manual checks, and [physical navigation](docs/PHYSICAL_NAVIGATION.md) for the
Jazzy launch, command routing and required supervised validation. Do not send
autonomous goals until the physical checks in that guide have passed. The
simulated camera plugin does not start a driver for a physical camera.
