"""Pi-side goal preview and optional Nav2 executor; motion is off by default.

NOT a safety-rated controller. The operator must provide an independent motor
power cutoff and verify Nav2, sensors, transforms, and the collision monitor.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_geometry_msgs import do_transform_point

from approach_geometry import approach_pose
from task_protocol import parse_task


class TaskExecutor(Node):
    def __init__(self):
        super().__init__('diffbot_ai_task_executor')
        self.declare_parameter('allow_motion', False)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('heartbeat_timeout', 2.0)
        self.declare_parameter('goal_timeout', 120.0)
        self.publisher = self.create_publisher(String, '/ai/status', 10)
        self.preview = self.create_publisher(PoseStamped, '/ai/goal_preview', 10)
        self.create_subscription(String, '/ai/task', self.on_task, 10)
        self.create_subscription(String, '/ai/heartbeat', self.on_heartbeat, 10)
        self.create_subscription(String, '/ai/detections', self.on_detection, 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(0.2, self.watchdog)
        self.heartbeats = {}
        self.detection = None
        self.received = None
        self.active = None
        self.seen = set()
        self.last_feedback = 0.0
        self.get_logger().warn('AI executor started. allow_motion=' + str(self.get_parameter('allow_motion').value))

    def report(self, state, detail, task_id=''):
        msg = String()
        msg.data = json.dumps({'id': task_id, 'state': state, 'detail': detail})
        self.publisher.publish(msg)
        self.get_logger().info(msg.data)

    def on_heartbeat(self, msg):
        if 1 <= len(msg.data) <= 64:
            self.heartbeats[msg.data] = time.monotonic()
        now = time.monotonic()
        self.heartbeats = {key: stamp for key, stamp in self.heartbeats.items() if now - stamp < 10.0}

    def fresh_detection(self):
        if self.detection is None or self.received is None:
            return False
        timeout = float(self.get_parameter('detection_timeout').value)
        try:
            stamp = self.detection['stamp']
            sensor_ns = int(stamp['sec']) * 1_000_000_000 + int(stamp['nanosec'])
            age = (self.get_clock().now().nanoseconds - sensor_ns) / 1e9
            return time.monotonic() - self.received <= timeout and -0.2 <= age <= timeout
        except (KeyError, TypeError, ValueError):
            return False

    def target_point(self, label):
        if not self.fresh_detection():
            raise ValueError('Camera detection is missing, stale, or clock is unsynchronized')
        data = self.detection
        candidates = [obj for obj in data['objects'] if obj.get('label') == label]
        if len(candidates) != 1:
            raise ValueError(f'Expected one {label}; found {len(candidates)}')
        xyz = candidates[0]['point_optical_m']
        if len(xyz) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz) or xyz[2] <= 0:
            raise ValueError('Invalid target position')
        point = PointStamped()
        point.header.frame_id = data['frame_id']
        point.header.stamp.sec = int(data['stamp']['sec'])
        point.header.stamp.nanosec = int(data['stamp']['nanosec'])
        point.point.x, point.point.y, point.point.z = [float(v) for v in xyz]
        return point

    def on_detection(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('objects'), list):
                return
            self.detection = data
            self.received = time.monotonic()
        except (ValueError, TypeError):
            return

    def on_task(self, msg):
        try:
            envelope = json.loads(msg.data)
            if not isinstance(envelope, dict) or set(envelope) != {'id', 'session', 'task'}:
                raise ValueError('Invalid envelope')
            request_id, session = envelope['id'], envelope['session']
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                raise ValueError('Invalid request ID')
            if not isinstance(session, str) or not 1 <= len(session) <= 64:
                raise ValueError('Invalid session')
            task = parse_task(json.dumps(envelope['task']))
            if task['task'] == 'stop':
                self.cancel('Operator stop')
                self.report('stopped', 'Stop requested', request_id)
                return
            if task['task'] == 'reject':
                raise ValueError('Unsupported instruction')
            if request_id in self.seen:
                raise ValueError('Duplicate request')
            self.seen.add(request_id)
            if len(self.seen) > 10000:
                raise ValueError('Request history full; restart executor while stopped')
            if self.active:
                raise ValueError('A task is already active; stop it first')
            heartbeat = self.heartbeats.get(session)
            if heartbeat is None or time.monotonic() - heartbeat > float(self.get_parameter('heartbeat_timeout').value):
                raise ValueError('Client heartbeat missing')
            # Nav2 is configured for 0.1 m/s; per-task speed limits are not wired yet.
            if not math.isclose(task['max_speed_mps'], 0.1, abs_tol=1e-9):
                raise ValueError('Per-task speed limits are not implemented; request 0.1 m/s')
            point = self.target_point(task['target'])
            stamp = Time.from_msg(point.header.stamp)
            map_frame = self.get_parameter('map_frame').value
            tf_camera = self.tf.lookup_transform(map_frame, point.header.frame_id, stamp, timeout=Duration(seconds=0.1))
            tf_robot = self.tf.lookup_transform(map_frame, self.get_parameter('base_frame').value, stamp, timeout=Duration(seconds=0.1))
            mapped = do_transform_point(point, tf_camera)
            robot = tf_robot.transform.translation
            goal = approach_pose((robot.x, robot.y), (mapped.point.x, mapped.point.y), task['stop_distance_m'])
            if goal is None:
                self.report('already_close', 'Base is already within requested standoff; no goal sent', request_id)
                return
            pose = PoseStamped()
            pose.header.frame_id = map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y, yaw = goal
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.preview.publish(pose)
            if not self.get_parameter('allow_motion').value:
                self.report('preview', 'Goal computed; motion disabled', request_id)
                return
            if not self.nav.server_is_ready():
                raise ValueError('Nav2 action server is unavailable')
            self.active = {'id': request_id, 'session': session, 'target': task['target'],
                           'deadline': time.monotonic() + float(self.get_parameter('goal_timeout').value),
                           'handle': None, 'cancel_requested': False}
            request = NavigateToPose.Goal()
            request.pose = pose
            future = self.nav.send_goal_async(
                request, feedback_callback=lambda f: self.on_feedback(f, request_id))
            future.add_done_callback(lambda f: self.goal_response(f, request_id))
            self.report('submitted', 'Nav2 goal sent', request_id)
        except (ValueError, TypeError, KeyError, IndexError, TransformException) as exc:
            self.report('rejected', str(exc))

    def on_feedback(self, message, request_id):
        now = time.monotonic()
        if not self.active or self.active['id'] != request_id or now - self.last_feedback < 2.0:
            return
        self.last_feedback = now
        distance = message.feedback.distance_remaining
        if math.isfinite(distance):
            self.report('navigating', f'Nav2 distance remaining: {distance:.2f} m', request_id)

    def goal_response(self, future, request_id):
        try:
            handle = future.result()
            active = self.active
            if active is None or active['id'] != request_id:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self.report('failed', 'Nav2 rejected the goal', request_id)
                self.active = None
                return
            active['handle'] = handle
            if active['cancel_requested']:
                handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(lambda f: self.goal_result(f, request_id))
        except Exception as exc:
            self.report('failed', f'Goal response error: {exc}', request_id)
            self.active = None

    def goal_result(self, future, request_id):
        try:
            status = future.result().status
            state = 'nav_goal_succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'finished'
            self.report(state, f'Nav2 result status: {status}; standoff not independently verified', request_id)
        except Exception as exc:
            self.report('failed', str(exc), request_id)
        if self.active and self.active['id'] == request_id:
            self.active = None

    def cancel(self, reason):
        if self.active and not self.active['cancel_requested']:
            self.active['cancel_requested'] = True
            if self.active['handle'] is not None:
                self.active['handle'].cancel_goal_async()
            self.report('cancel_requested', reason, self.active['id'])

    def watchdog(self):
        if not self.active or self.active['cancel_requested']:
            return
        now = time.monotonic()
        heartbeat = self.heartbeats.get(self.active['session'])
        if heartbeat is None or now - heartbeat > float(self.get_parameter('heartbeat_timeout').value):
            self.cancel('Client heartbeat timed out')
            return
        if now > self.active['deadline']:
            self.cancel('Navigation deadline exceeded')
            return
        try:
            self.target_point(self.active['target'])
        except (ValueError, TypeError, KeyError, IndexError):
            self.cancel('Target detection lost, stale or ambiguous')


def main():
    rclpy.init()
    node = TaskExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel('Executor shutting down')
        # Allow cancellation messages to be dispatched before shutdown.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
