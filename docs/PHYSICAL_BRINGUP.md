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
6. Add the real IMU and LiDAR drivers before SLAM or autonomous navigation.

The physical robot should not run SLAM or Nav2 from simulated LiDAR data. A
simulated LiDAR is useful only in Gazebo; the real robot needs a real range
sensor, or a different localization/perception design.
