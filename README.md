# Custom DiffBot: ROS 2 Navigation

A custom differential-drive robot simulation built with ROS 2 Humble, Gazebo Classic, RViz, and Nav2. The project includes a Xacro robot model, simulated camera and 2D LiDAR, differential-drive odometry, a Gazebo world, and robot-specific Nav2 parameters.

## Features

- Two-wheel differential drive with a caster wheel
- Simulated 360° LiDAR publishing `/scan`
- Simulated camera
- Odometry and TF published by the Gazebo drive plugin
- AMCL localization and Nav2 navigation using a saved map
- Custom robot footprint and local/global costmap configuration

## Packages

| Package | Contents |
| --- | --- |
| `my_robot_description` | Xacro model, sensors, and robot visualization |
| `my_robot_bringup` | Gazebo world, launch files, RViz configuration, and Nav2 parameters |

The TF chain used for navigation is:

map → odom → base_footprint → base_link → lidar_link


AMCL provides `map → odom`; Gazebo provides `odom → base_footprint`.

## Requirements

- ROS 2 Humble
- Gazebo Classic and `gazebo_ros`
- Nav2
- RViz 2
- `xacro`
- SLAM Toolbox, if you want to create a new map

## Build the file
FIrst delete the build install and log folder

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_Nav2_diffbot
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```

This starts Gazebo, spawns the robot, publishes its description, and opens RViz. To run Gazebo without its graphical client:

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml gui:=false
```

## Create a map

With the simulation running, start SLAM Toolbox in another terminal:

```bash
source ~/ros2_Nav2_diffbot/install/setup.bash
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
source ~/ros2_Nav2_diffbot/install/setup.bash
ros2 launch my_robot_bringup navigation.launch.py \
  map:=$HOME/my_maps/my_world.yaml
```

In RViz, set **Fixed Frame** to `map`, use **2D Pose Estimate** to set the robot's starting position and heading, then send a **2D Goal Pose**.

A saved map is required for this launch and is not included in the repository. The map must match the Gazebo world: mismatched walls or an incorrect initial pose can cause AMCL pose jumps and misaligned costmaps.

In the Extras folder, I have saved a template file to manipulate the robot through without using RViz 2D Goal pose buttons, u can run the python files with your coordinates and it will run automatically.

## Robot configuration

| Parameter | Value |
| --- | --- |
| Wheel separation | 0.45 m |
| Wheel diameter | 0.20 m |
| Odometry update rate | 50 Hz |
| LiDAR update rate | 10 Hz |
| LiDAR range | 0.12–3.5 m |
| LiDAR samples | 360 per scan |

The current project is a **simulation prototype**. Its LiDAR, camera, odometry, and motor control use Gazebo plugins; physical hardware drivers and localization without a physical LiDAR are not implemented.
