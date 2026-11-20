"""Interactive laptop task client. Does not publish velocity commands."""

import json
import threading
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ai_task_preview import interpret


class TaskClient(Node):
    def __init__(self):
        super().__init__('diffbot_ai_task_client')
        self.requests = self.create_publisher(String, '/ai/task', 10)
        self.heartbeats = self.create_publisher(String, '/ai/heartbeat', 10)
        self.create_subscription(String, '/ai/status', self.on_status, 10)
        self.create_timer(0.5, self.heartbeat)
        self.session = uuid.uuid4().hex

    def heartbeat(self):
        msg = String()
        msg.data = self.session
        self.heartbeats.publish(msg)

    def on_status(self, msg):
        print('\nRobot status:', msg.data, flush=True)

    def send(self, task):
        msg = String()
        msg.data = json.dumps({'id': uuid.uuid4().hex, 'session': self.session, 'task': task})
        self.requests.publish(msg)
        print('Sent:', msg.data, flush=True)


def main():
    rclpy.init()
    node = TaskClient()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print('Type stop for immediate cancellation; type quit to exit.')
    try:
        while rclpy.ok():
            instruction = input('Instruction: ').strip()
            if instruction.lower() in ('quit', 'exit'):
                break
            if not instruction:
                continue
            if instruction.lower() == 'stop':
                task = {'task': 'stop', 'target': '', 'stop_distance_m': 0.5, 'max_speed_mps': 0.1}
            else:
                try:
                    task = interpret(instruction, 'qwen2.5:1.5b', 'http://127.0.0.1:11434/api/chat')
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    print(f'Task rejected: {exc}')
                    continue
            print('Interpreted:', json.dumps(task))
            if task['task'] == 'approach_object':
                if input('Send request to robot? [y/N] ').strip().lower() != 'y':
                    continue
            node.send(task)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        # A lost heartbeat also requests cancellation on the Pi.
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
