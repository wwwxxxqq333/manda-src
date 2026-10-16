#!/usr/bin/env python3
"""Action server for SideManoeuvre (keeps service for compatibility).

Goal: vy, vx, duration
Feedback: progress (0.0-1.0)
Result: success, message
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from std_msgs.msg import Int8
from geometry_msgs.msg import Twist
from manda_description.action import SideManoeuvre


class SideManoeuvreActionServer(Node):
    def __init__(self):
        super().__init__('side_manoeuvre_action_server')

        self._action_server = ActionServer(
            self,
            SideManoeuvre,
            'side_manoeuvre_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.drive_mode_pub = self.create_publisher(Int8, '/drive_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cancel_requested = False

        self.get_logger().info('SideManoeuvre Action Server ready (action: /side_manoeuvre_action)')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received SideManoeuvre goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        vy = float(goal.vy)
        vx = float(goal.vx)
        duration = float(goal.duration)

        # publish drive_mode=1 (OMNI)
        m = Int8(); m.data = 1
        self.drive_mode_pub.publish(m)

        start = time.time()
        rate = 20.0
        interval = 1.0 / rate
        while rclpy.ok() and (time.time() - start) < duration:
            if self._cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                # stop and restore
                self.cmd_vel_pub.publish(Twist())
                m2 = Int8(); m2.data = 0
                self.drive_mode_pub.publish(m2)
                result = SideManoeuvre.Result()
                result.success = False
                result.message = 'Canceled'
                return result

            t = time.time() - start
            progress = t / duration
            # publish cmd_vel
            twist = Twist(); twist.linear.x = vx; twist.linear.y = vy; twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            # feedback
            feedback = SideManoeuvre.Feedback()
            feedback.progress = float(progress)
            goal_handle.publish_feedback(feedback)
            time.sleep(interval)

        # stop and restore normal driving (drive_mode=0)
        self.cmd_vel_pub.publish(Twist())
        m2 = Int8(); m2.data = 0
        self.drive_mode_pub.publish(m2)

        result = SideManoeuvre.Result()
        result.success = True
        result.message = 'Completed'
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SideManoeuvreActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
