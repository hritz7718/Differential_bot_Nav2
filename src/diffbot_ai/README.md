# DiffBot AI integration (experimental)

This is an unverified prototype, not a safety-rated robot controller. Motion is
disabled by default. Do not enable it on physical hardware until an independent,
readily accessible motor-power emergency stop is fitted and all tests below have
been performed. Unplugging a wire is not an adequate emergency stop.

## Architecture

* Pi, ROS 2 Jazzy: RealSense driver, robot bringup, Nav2, collision monitor,
  `task_executor.py`.
* Laptop, ROS 2 Jazzy: `rgbd_detection.py`, `task_client.py`, optionally RViz.
* Laptop: Ollama and Qwen2.5, accessible to the task client at
  `http://127.0.0.1:11434` (for example, a host-networked Jazzy container).

`/ai/detections` and `/ai/task` contain JSON in `std_msgs/msg/String`;
`/ai/heartbeat` and `/ai/status` are also String. `/ai/goal_preview` is
`geometry_msgs/msg/PoseStamped`. The executor uses timestamped TF lookups,
and sends an optional `nav2_msgs/action/NavigateToPose` goal. It never sends
velocity commands. Inputs have no authentication: use only an isolated,
trusted network and do not assume ROS domain isolation is security.

The existing Nav2 collision monitor is not safety-rated. Detection freshness,
heartbeat and Nav2 cancellation are best-effort software checks, not emergency
stops. A killed executor, DDS failure, or unresponsive Nav2 may prevent
cancellation. The approach point is based on the base origin, not the robot
footprint or the object's full geometry. The final distance is not verified.
This prototype does not track objects, replan a moved target, search for a
missing object, or ensure the requested speed independently of Nav2 parameters.
Only the existing configured 0.1 m/s speed is accepted.

## Requirements

* Pi: ROS 2 Jazzy, existing built DiffBot workspace, Nav2, RPLIDAR driver,
  `realsense2_camera`, librealsense, `tf2_geometry_msgs` and Nav2 Python messages.
* Laptop Jazzy environment: `rclpy`, `sensor_msgs`, `geometry_msgs`,
  `nav2_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, `message_filters`,
  NumPy, Ultralytics and a YOLO11n weights file. Ollama and Qwen2.5 run on the
  laptop and must be reachable from the client process.
* Both machines: synchronized system clocks and network access between Jazzy
  ROS nodes. Stream rates, latency, memory and QoS must be measured on your
  network. Match the RealSense driver's parameters to its installed version.

## Essential camera prerequisites

The existing URDF's `camera_link_optical` is only a nominal transform. Compare
it to the D435 driver frame IDs and physically measured camera pose. This launch
sets `publish_tf:=false` to avoid duplicate TF publishers; you must explicitly
provide one correct, timestamped TF chain from `base_footprint` through the
actual camera optical frame before preview goals work. Do not simply rename a
frame: verify translations, orientation, axes and physical calibration. Obtain
the driver-provided CameraInfo for the actual color profile.

The perception node assumes a *rectified* color image and a depth image aligned
to that same rectified color geometry. `CameraInfo.P` is used for deprojection;
raw distorted pixels are not supported. Some driver installations may not
publish `image_rect_raw` or may align depth to unrectified color. Configure an
appropriate rectification/alignment pipeline and adjust the launch topic
arguments. Do not use estimated object points until this is verified.

## Preview-only procedure

These commands are provided for the operator to run; they have not been tested
in this environment. Keep motor power disconnected.

1. On the Pi, source Jazzy and the DiffBot workspace, then start the camera and
   executor. This does not start robot bringup by default:

   ```bash
   cd ~/ros2_Nav2_diffbot
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch "$PWD/src/diffbot_ai/ai_pi.launch.py"
   ```

2. Start your existing localization and Nav2 separately when the Pi, LiDAR,
   serial connection, and map are ready. For a combined launch, stop the
   previous launch and use the command below with your real paths. The executor
   still defaults to preview-only:

   ```bash
   ros2 launch "$PWD/src/diffbot_ai/ai_pi.launch.py" \
     start_navigation:=true map:=/absolute/path/to/map.yaml \
     serial_port:=/dev/serial/by-id/YOUR_ARDUINO \
     lidar_serial_port:=/dev/serial/by-id/YOUR_LIDAR
   ```

3. In the laptop Jazzy environment, with this repository mounted at the same
   workspace path and its Python environment configured:

   ```bash
   cd ~/ros2_Nav2_diffbot
   source /opt/ros/jazzy/setup.bash
   ros2 launch "$PWD/src/diffbot_ai/ai_laptop.launch.py" \
     model:=/absolute/path/to/yolo11n.pt
   ```

   Override `color_topic`, `depth_topic`, `camera_info_topic` and `python` on
   the launch command if needed. A virtualenv Python must also be able to import
   the Jazzy ROS Python packages and use a compatible system Python version.

4. In another laptop Jazzy terminal, start the interactive client:

   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 ~/ros2_Nav2_diffbot/src/diffbot_ai/task_client.py
   ```

   The client asks for confirmation before submitting approach tasks. The word
   `stop` bypasses Ollama and requests cancellation. A lost client heartbeat,
   stale or ambiguous target, or goal deadline also requests cancellation.
   Since a synchronous Ollama request can block interactive input, use a
   separate terminal for an immediate software stop if it is busy:

   ```bash
   ros2 topic pub --once /ai/task std_msgs/msg/String \
     '{data: "{\"id\":\"manual-stop\",\"session\":\"operator\",\"task\":{\"task\":\"stop\",\"target\":\"\",\"stop_distance_m\":0.5,\"max_speed_mps\":0.1}}"}'
   ```

   This is not an emergency stop and cannot be relied upon during network loss.

5. Inspect the pipeline:

   ```bash
   ros2 topic list | grep -E 'camera|ai|scan|odom|map'
   ros2 topic hz /camera/camera/color/image_rect_raw
   ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
   ros2 topic echo /ai/detections
   ros2 topic echo /ai/status
   ros2 topic echo /ai/goal_preview
   ros2 run tf2_ros tf2_echo map camera_link_optical
   ```

   The last frame name is an example: use the actual detection `frame_id`.
   With no valid transform, fresh detection, localization or client heartbeat,
   the task is rejected. Multiple same-class detections are rejected as
   ambiguous. RViz must run in a Jazzy environment and can display the normal
   robot, TF, scan, map and Nav2 topics plus `/ai/goal_preview` as Pose.

## Before any physical motion

Independently verify emergency stop, LiDAR coverage, collision monitor routing,
motor direction, odometry, AMCL localization, camera calibration, clock sync,
target detection robustness, footprint clearance, speed limits, Nav2 cancellation,
and failure behavior. Do not use `allow_motion:=true` until these have been
verified in simulation and supervised hardware testing. Motion is explicitly
opt-in; the flag is not authorization or evidence of safety.
