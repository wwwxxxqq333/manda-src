#!/usr/bin/env python3
"""
FWIDS low-level validation: command all 8 motors directly.
Usage:
  ros2 run manda_description test_all_wheels.py forward 5.0
  ros2 run manda_description test_all_wheels.py stop
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]


class WheelTester(Node):
    def __init__(self):
        super().__init__("wheel_tester")
        self.steer_pubs = {}
        self.rotor_pubs = {}
        for name in WHEELS:
            self.steer_pubs[name] = self.create_publisher(
                Float64MultiArray, f"/{name}_steer_controller/commands", 10)
            self.rotor_pubs[name] = self.create_publisher(
                Float64MultiArray, f"/{name}_rotor_controller/commands", 10)

    def wait_for_subscribers(self, timeout=5.0):
        """Spin until all publishers have at least one subscriber."""
        all_pubs = list(self.steer_pubs.values()) + list(self.rotor_pubs.values())
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout
        while rclpy.ok():
            unmatched = [p for p in all_pubs if p.get_subscription_count() == 0]
            if not unmatched:
                self.get_logger().info("All publishers matched")
                return True
            now = self.get_clock().now().nanoseconds / 1e9
            if now > deadline:
                self.get_logger().warn(
                    f"{len(unmatched)} publishers unmatched after {timeout}s")
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def all_steer(self, angle: float):
        msg = Float64MultiArray()
        msg.data = [angle]
        for name in WHEELS:
            self.steer_pubs[name].publish(msg)
        self.get_logger().info(f"Steer all → {angle:.2f} rad")

    def all_rotor(self, velocity: float):
        msg = Float64MultiArray()
        msg.data = [velocity]
        for name in WHEELS:
            self.rotor_pubs[name].publish(msg)
        self.get_logger().info(f"Rotor all → {velocity:.2f} rad/s")

    def forward(self, speed: float):
        self.all_steer(0.0)
        self.all_rotor(speed)

    def stop(self):
        self.all_rotor(0.0)


def main(args=None):
    rclpy.init(args=args)
    tester = WheelTester()

    cmd = sys.argv[1] if len(sys.argv) > 1 else "forward"
    val = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    # Wait for controller subscribers to discover us
    if not tester.wait_for_subscribers(timeout=5.0):
        tester.get_logger().error(
            "Controllers not reachable. Is Gazebo running?")
        tester.destroy_node()
        rclpy.shutdown()
        return

    if cmd == "forward":
        tester.forward(val)
    elif cmd == "stop":
        tester.stop()
    elif cmd == "steer":
        tester.all_steer(val)
    elif cmd == "rotor":
        tester.all_rotor(val)
    else:
        tester.get_logger().error(f"Unknown cmd: {cmd}")

    # Deliver messages before exit
    for _ in range(20):
        rclpy.spin_once(tester, timeout_sec=0.05)

    tester.destroy_node()
    rclpy.shutdown()
