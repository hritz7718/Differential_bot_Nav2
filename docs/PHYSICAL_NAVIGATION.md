# Physical Navigation: Supervised Bringup

This is an integration starting point, not a declaration that the robot is safe
for autonomous driving. Run on the Raspberry Pi with ROS 2 Jazzy, a real A1
scan, calibrated encoders/odometry, and a saved map of the current environment.
Do not run SLAM and AMCL simultaneously, and do not start a second copy of the
physical bringup. Verify that the installed Nav2 Jazzy bringup actually starts
and activates `collision_monitor`; versions with different launch wiring may
not do so.

## Command and feedback paths

The physical navigation launch sets the hardware bridge input to
`/cmd_vel_safe`:

```text
Nav2 controller / behavior -> /cmd_vel_nav -> velocity_smoother
-> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_safe
-> twist_stamper -> /diff_drive_controller/cmd_vel -> Arduino

encoders -> /joint_states and /diff_drive_controller/odom
LiDAR -> /scan -> AMCL, costmaps, collision_monitor
```

The bridge's default input in standalone `physical_bringup` remains `/cmd_vel`
for manual teleop. In `physical_navigation`, publishing to `/cmd_vel` no longer
drives the bridge; manual control requires a separate deliberate workflow.
Do not publish directly to `/cmd_vel_safe` or the wheel-controller input: that
would bypass the monitor. ROS topics are not an access-control boundary, and
the monitor is not a safety-rated emergency stop. Docking is not validated and
must not be used.

The physical profile limits Nav2 to 0.10 m/s and 0.5 rad/s, blocks the global
planner from unknown map space, and uses `/diff_drive_controller/odom`. The
robot footprint is based on the URDF and must be measured against the complete
physical robot, including wires, wheels, mounts and any projecting parts. The
stop polygon and minimum scan points are starting estimates, not measured
stopping distances. A single planar LiDAR cannot detect every obstacle,
overhang, drop-off, transparent surface, or object outside its scan plane.

## Before powering the motors

1. Verify the LiDAR mount and yaw, scan alignment, and Pi clock.
2. Verify wheel directions and encoder signs with the wheels raised. Calibrate
   counts/revolution, effective wheel radius, separation, and motor PI gains.
3. Check the L298N wiring, supply voltage/current, temperature, and USB power.
   Verify a readily accessible physical motor-power cutoff. The optional
   firmware stop input is disabled and is not a safety-rated emergency stop.
4. Confirm the map depicts the actual environment and its origin/orientation.
5. Confirm all command sources: publishing directly to the controller can
   bypass the collision monitor. Do not run other command publishers or a
   second robot controller during the test.

## Start and inspect with motor power off

Make sure the latest code and `sllidar_ros2` driver are built on the Pi. Replace
the example paths with the actual map and stable device paths:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_Nav2_diffbot/install/setup.bash
ros2 launch my_robot_bringup physical_navigation.launch.py \
  map:=$HOME/my_maps/physical_map.yaml \
  serial_port:=/dev/serial/by-id/YOUR_ARDUINO_MEGA \
  lidar_serial_port:=/dev/serial/by-id/YOUR_RPLIDAR
```

In another sourced terminal:

```bash
ros2 control list_controllers
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /collision_monitor
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic hz /diff_drive_controller/odom
ros2 topic info /cmd_vel_smoothed -v
ros2 topic info /cmd_vel_safe -v
ros2 topic info /diff_drive_controller/cmd_vel -v
ros2 run tf2_ros tf2_echo odom base_footprint
```

The controllers and lifecycle nodes must be active. The safe-topic publisher
must be the collision monitor, and its subscriber the twist stamper; the
controller input must have only intended publishers. In RViz, set fixed frame
`map`, display `/map` and `/scan`, and use 2D Pose Estimate at the robot's
actual pose. Check `ros2 run tf2_ros tf2_echo map odom` after localization.
The scans must align with the saved map before accepting a goal.

## Supervised safety validation

With motor power disconnected, test loss of `/scan`, the collision stop zone,
and the command chain; check that nonzero commands do not reach the bridge when
the monitor is stopped or no scan is available. Validate these behaviours
again in a safe simulator or test fixture before floor use. Then, only in a
cleared area with the robot attended and a power cutoff in hand, measure actual
stopping distance at the intended speeds, test loss of scan/USB/commands, and
verify localization stability. Adjust limits and the stop zone based on those
measurements, with a conservative margin. Do not send autonomous goals until
these checks pass.
