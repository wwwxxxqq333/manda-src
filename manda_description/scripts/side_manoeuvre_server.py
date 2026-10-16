#!/usr/bin/env python3
"""Side manoeuvre service: called to perform precise lateral slide.

Service: /side_manoeuvre (example_interfaces/srv/SetBool)
  - request.data == True : start manoeuvre
  - request.data == False: cancel manoeuvre

Behaviour: when started, publish /drive_mode=1, then publish a short cmd_vel
sequence (configurable duration, vy, vx) at `rate` Hz, then publish /drive_mode=2
to clear override. Returns success when sequence completes or was cancelled.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool


class SideManoeuvreServer(Node):
    def __init__(self):
        super().__init__('side_manoeuvre_server')

        # publishers
        self.drive_mode_pub = self.create_publisher(Int8, '/drive_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # service
        self.srv = self.create_service(SetBool, '/side_manoeuvre', self.handle_side)

        # params
        self.declare_parameter('vy', 0.2)
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('duration', 1.5)
        self.declare_parameter('rate', 20)

        self._running = False

        self.get_logger().info('Side manoeuvre service ready (/side_manoeuvre)')

    def handle_side(self, request, response):
        if request.data:
            if self._running:
                response.success = False
                response.message = 'Manoeuvre already running'
                return response

            vy = float(self.get_parameter('vy').value)
            vx = float(self.get_parameter('vx').value)
            duration = float(self.get_parameter('duration').value)
            rate = float(self.get_parameter('rate').value)

            self.get_logger().info(f'Starting side manoeuvre: vy={vy} vx={vx} dur={duration}s')
            self._running = True

            # 1) set drive_mode = 1 (force OMNI)
            m = Int8()
            m.data = 1
            self.drive_mode_pub.publish(m)

            # 2) publish cmd_vel for duration
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = 0.0

            interval = 1.0 / max(1.0, rate)
            t0 = time.time()
            while self._running and (time.time() - t0) < duration:
                self.cmd_vel_pub.publish(twist)
                time.sleep(interval)

            # 3) stop cmd_vel
            stop = Twist()
            self.cmd_vel_pub.publish(stop)

            # 4) restore normal driving mode (drive_mode = 0 -> Ackermann)
            m2 = Int8()
            m2.data = 0
            self.drive_mode_pub.publish(m2)

            self._running = False
            response.success = True
            response.message = 'Manoeuvre completed'
            return response
        else:
            # cancel request
            if self._running:
                self.get_logger().info('Cancelling side manoeuvre')
                self._running = False
                # ensure stop and restore normal mode
                self.cmd_vel_pub.publish(Twist())
                m2 = Int8(); m2.data = 0
                self.drive_mode_pub.publish(m2)
                response.success = True
                response.message = 'Cancelled'
            else:
                response.success = False
                response.message = 'No manoeuvre running'
            return response


def main(args=None):
    rclpy.init(args=args)
    node = SideManoeuvreServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
