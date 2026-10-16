#!/usr/bin/env python3
"""Feeder manager: monitors robot position and triggers side manoeuvre when
approaching the configured feeding point.

Publishes:
  - /drive_mode (Int8) periodically: 0 = normal Ackermann
Calls:
  - /side_manoeuvre (SetBool) to perform lateral slide when within trigger distance

Parameters:
  - feed_x (float)
  - feed_y (float)
  - trigger_distance (float)  # start manoeuvre when closer than this
  - cooldown (float)  # seconds between maneuvers
  - publish_rate (float) # Hz for publishing drive_mode=0 when idle

"""

import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseWithCovarianceStamped
from example_interfaces.srv import SetBool


class FeederManager(Node):
    def __init__(self):
        super().__init__('feeder_manager')

        # params
        self.declare_parameter('feed_x', 0.0)
        self.declare_parameter('feed_y', 0.0)
        self.declare_parameter('trigger_distance', 1.5)
        self.declare_parameter('cooldown', 10.0)
        self.declare_parameter('publish_rate', 1.0)

        self.feed_x = float(self.get_parameter('feed_x').value)
        self.feed_y = float(self.get_parameter('feed_y').value)
        self.trigger_distance = float(self.get_parameter('trigger_distance').value)
        self.cooldown = float(self.get_parameter('cooldown').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.drive_mode_pub = self.create_publisher(Int8, '/drive_mode', 10)

        # try to subscribe to AMCL pose; if not present, user can modify
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_cb,
            10)

        self._last_manoeuvre = 0.0
        self._busy = False

        # service client
        self.cli = self.create_client(SetBool, '/side_manoeuvre')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /side_manoeuvre service...')

        # timer to publish drive_mode=0 when idle
        self.create_timer(1.0 / max(0.1, self.publish_rate), self.pub_drive_mode)

        self.get_logger().info('FeederManager ready — feed at (%.2f, %.2f)' % (self.feed_x, self.feed_y))

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if self._busy:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dx = x - self.feed_x
        dy = y - self.feed_y
        dist = math.hypot(dx, dy)
        # if within trigger distance and cooldown expired
        if dist <= self.trigger_distance and (time.time() - self._last_manoeuvre) > self.cooldown:
            self.get_logger().info(f'Approaching feed point (d={dist:.2f}m) — triggering side manoeuvre')
            self._busy = True
            req = SetBool.Request()
            req.data = True
            future = self.cli.call_async(req)

            # wait for service result synchronously but without blocking other callbacks
            while rclpy.ok() and not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)

            if future.done() and future.result() is not None:
                res = future.result()
                self.get_logger().info('Side manoeuvre result: %s - %s' % (res.success, res.message))
            else:
                self.get_logger().warn('Side manoeuvre service call failed or timed out')

            self._last_manoeuvre = time.time()
            self._busy = False

    def pub_drive_mode(self):
        if not self._busy:
            m = Int8(); m.data = 0
            self.drive_mode_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FeederManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
