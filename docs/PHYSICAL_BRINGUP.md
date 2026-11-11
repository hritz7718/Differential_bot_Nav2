# Physical DiffBot Bringup

This stage connects the Raspberry Pi 5, Arduino Mega, wheel encoders, and ADIY
L293D module through `ros2_control`. Keep the wheels clear of the floor until
all direction checks pass.

## Configured dimensions

| Item | Value |
| --- | ---: |
| Wheel diameter | 68.5 mm |
| Wheel separation | 285 mm |
| Driven axle to caster center | 193 mm |
| Chassis | 310 x 260 x 45 mm |
| Chassis top above floor | 93 mm |
| Driven axle to chassis front | 225 mm |
| Encoder count used by software | 450 counts/wheel revolution |

The wheel width (26 mm), wheel mass (25 g each), and caster radius (15 mm) are
temporary estimates. Measure them before final inertia and footprint tuning.

## Wiring

ADIY L293D logic header order is `A1 A2 EN_A VCC GND EN_B B2 B1`.

| L293D | Mega pin | Purpose |
| --- | ---: | --- |
| A1 | 7 | Right motor direction 1 |
| A2 | 8 | Right motor direction 2 |
| EN_A | 5 | Right motor PWM |
| VCC | 5V | Driver logic power |
| GND | GND | Common logic ground |
| EN_B | 6 | Left motor PWM |
| B2 | 10 | Left motor direction 2 |
| B1 | 9 | Left motor direction 1 |

Connect the right motor to `MA` and the left motor to `MB`. On both outputs,
connect red to `M1` and white to `M2` as specified. Initial software polarity
is set for mirrored motors and must still be verified.

| Encoder | Mega pin |
| --- | ---: |
| Right A | 2 |
| Right B | 22 |
| Left A | 3 |
| Left B | 23 |

Encoder power goes to Mega 5V/GND if the encoder is rated for 5V. Do not wire
encoder outputs directly to Raspberry Pi GPIO. The optional normally-closed
emergency-stop input is pin 18; it remains disabled in firmware until wired.

Motor power goes to the L293D `V+` and `V-` terminals from a suitable motor
supply. Never power the motors from the Mega or Pi 5V rail. The motor supply,
L293D, Mega, and Pi USB connection must share ground. Add a fuse and a physical
power switch before floor testing.

## Firmware

Open `firmware/diffbot_mega/diffbot_mega.ino` in Arduino IDE, select **Arduino
Mega or Mega 2560**, select the correct port, and upload it.

The initial firmware limits PWM to `90/255`, stops after 250 ms without a valid
command, and checks every serial packet with CRC-16. PID values are starting
values only. Watch L293D and motor temperature closely; stop immediately if the
driver overheats or resets.

## Build on Raspberry Pi

Install the Jazzy control packages if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

Allow serial access once, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Build:

```bash
cd ~/ros2_Nav2_diffbot
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Find the stable Mega device path:

```bash
ls -l /dev/serial/by-id/
```

Launch with that full path:

```bash
ros2 launch my_robot_bringup physical_bringup.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ARDUINO_MEGA
```

## RPLIDAR A1

The physical launch starts Slamtec's `sllidar_ros2` driver by default. The
LiDAR serial port is independent of the Arduino port. On the Raspberry Pi,
install the driver from the official repository if it is not already available:

```bash
cd ~/ros2_Nav2_diffbot
git clone https://github.com/Slamtec/sllidar_ros2.git src/sllidar_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to my_robot_bringup sllidar_ros2
source install/setup.bash
ros2 pkg executables sllidar_ros2
ls -l /dev/serial/by-id/
```

If the driver is already installed, do not clone a second copy. The user must
have read/write access to both serial devices; membership in the `dialout`
group usually requires logging out and back in. Do not use `chmod 777` on the
device. Identify the RPLIDAR USB adapter separately from the Arduino, then pass
its stable path:

```bash
ros2 launch my_robot_bringup physical_bringup.launch.py \
  serial_port:=/dev/serial/by-id/YOUR_ARDUINO_MEGA \
  lidar_serial_port:=/dev/serial/by-id/YOUR_RPLIDAR
```

The defaults are `/dev/ttyUSB0` and `115200` baud for the A1; both can be
overridden using `lidar_serial_port` and `lidar_baud_rate`. For a base-only
test, pass `use_lidar:=false`.

With the launch running, check on the Pi and the RViz computer:

```bash
ros2 node list | grep sllidar
ros2 topic info /scan -v
ros2 topic hz /scan --qos-reliability best_effort
ros2 topic echo --once /scan --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link lidar_link
```

The scan header should use `lidar_link`. In RViz add a LaserScan display for
`/scan`, select a connected fixed frame such as `odom`, and use a compatible
reliability setting (Best Effort is compatible with either publisher policy).
The URDF provides the fixed `base_link` to `lidar_link` transform; do not
publish a second transform for it. Its position and zero rotation are model
assumptions: measure the actual sensor location and yaw, and update the joint
in `src/my_robot_description/urdf/lidar.xacro` if needed. Synchronize the Pi
and laptop clocks when using real time.

If `/scan` is absent on the Pi, inspect the `sllidar_node` output, check that
the port exists and is readable, and check that no other process owns it with
`fuser -v /dev/ttyUSB0` (substitute the actual device). A spinning LiDAR alone
does not prove serial data works. If it appears on the Pi but not the laptop,
check ROS domain, DDS discovery, network, and RViz QoS. Investigate USB power
or over-current errors before reconnecting hardware.

## Raised-wheel test

In a second sourced terminal:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /odom
```

Send a slow forward command for a few seconds:

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Stop the publisher with `Ctrl+C`; the robot must stop within 250 ms. Both
wheels must rotate in the robot's forward direction and both encoder positions
must increase. If a motor is reversed, change its `k*MotorSign`. If its encoder
count is reversed, change its `k*EncoderSign`. Do not swap signs at random or
hide an encoder error by changing the ROS wheel joint definitions.

Verify the encoder count by marking a wheel, turning it exactly ten revolutions,
and dividing the tick change by ten. Replace `450` in both the firmware and
`ros2_control.xacro` if the measured value differs.

## Calibration order

1. Confirm motor direction and encoder sign with the wheels raised.
2. Measure encoder counts over ten wheel revolutions.
3. Tune feed-forward and PID at several wheel speeds.
4. Drive a measured straight distance and tune wheel-radius multipliers.
5. Rotate through several full turns and tune wheel-separation multiplier.
6. Verify the real LiDAR driver and add the IMU driver before SLAM or navigation.

The physical robot should not run SLAM or Nav2 from simulated LiDAR data. A
simulated LiDAR is useful only in Gazebo; the real robot needs a real range
sensor, or a different localization/perception design.
