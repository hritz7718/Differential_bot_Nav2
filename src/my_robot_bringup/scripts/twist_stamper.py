#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.publisher = self.create_publisher(
            TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.stamp_command, 10)

    def stamp_command(self, command):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_footprint'
        stamped.twist = command
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
