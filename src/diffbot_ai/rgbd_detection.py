"""Publish YOLO detections with aligned RGB-D coordinates; does not move the robot.

Run with ROS 2 Jazzy and an environment containing ultralytics and cv_bridge.
Input depth must be aligned with the rectified color image.
"""

import json
import time

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO

from approach_geometry import deproject_rectified


class RgbdDetection(Node):
    def __init__(self):
        super().__init__('diffbot_rgbd_detection')
        self.declare_parameter('color_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.45)
        self.declare_parameter('period_seconds', 0.5)
        self.declare_parameter('sync_slop_seconds', 0.06)
        self.info = None
        self.bridge = CvBridge()
        self.last_inference = 0.0
        self.model = YOLO(self.get_parameter('model').value)
        self.publisher = self.create_publisher(String, '/ai/detections', 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.on_info, qos_profile_sensor_data)
        self.color_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('color_topic').value,
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value,
            qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], 5,
            self.get_parameter('sync_slop_seconds').value)
        self.sync.registerCallback(self.on_images)
        self.get_logger().info('Publishing camera-frame detections to /ai/detections')

    def on_info(self, msg):
        self.info = msg

    def on_images(self, color, depth):
        now = time.monotonic()
        if now - self.last_inference < self.get_parameter('period_seconds').value:
            return
        self.last_inference = now
        info = self.info
        if info is None or color.width != depth.width or color.height != depth.height:
            self.get_logger().warn('Waiting for aligned RGB/depth and camera info', throttle_duration_sec=5.0)
            return
        if info.width != color.width or info.height != color.height:
            self.get_logger().warn('CameraInfo dimensions do not match image', throttle_duration_sec=5.0)
            return
        if not color.header.frame_id or info.header.frame_id != color.header.frame_id:
            self.get_logger().warn('Color image and CameraInfo frame IDs differ', throttle_duration_sec=5.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            if depth.encoding == '16UC1':
                scale = 0.001
            elif depth.encoding == '32FC1':
                scale = 1.0
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {depth.encoding}', throttle_duration_sec=5.0)
                return
            # P describes the rectified camera; for unrectified images this is not sufficient.
            fx, fy, cx, cy = info.p[0], info.p[5], info.p[2], info.p[6]
            if fx <= 0 or fy <= 0:
                return
            results = self.model(image, conf=float(self.get_parameter('confidence').value), verbose=False)[0]
            objects = []
            for box in results.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                half_w, half_h = (x2 - x1) * 0.2, (y2 - y1) * 0.2
                xa, xb = max(0, int(u - half_w)), min(color.width, int(u + half_w))
                ya, yb = max(0, int(v - half_h)), min(color.height, int(v + half_h))
                roi = np.asarray(depth_image[ya:yb, xa:xb], dtype=np.float32) * scale
                valid = roi[np.isfinite(roi) & (roi >= 0.15) & (roi <= 8.0)]
                if valid.size < 20:
                    continue
                z = float(np.median(valid))
                point = deproject_rectified(u, v, z, fx, fy, cx, cy)
                objects.append({'label': str(self.model.names[int(box.cls[0])]),
                                'confidence': float(box.conf[0]),
                                'point_optical_m': list(point),
                                'bbox_xyxy': [x1, y1, x2, y2]})
            output = {'frame_id': color.header.frame_id,
                      'stamp': {'sec': color.header.stamp.sec, 'nanosec': color.header.stamp.nanosec},
                      'objects': objects}
            message = String()
            message.data = json.dumps(output, allow_nan=False)
            self.publisher.publish(message)
        except (CvBridgeError, ValueError, TypeError, RuntimeError) as exc:
            self.get_logger().error(f'Failed to process RGB-D frame: {exc}', throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = RgbdDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
