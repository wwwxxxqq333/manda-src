#!/usr/bin/env python3
"""
imu_converter.py — 速腾雷达 IMU 数据转发/监控节点

订阅: /rslidar_imu_data (sensor_msgs/Imu)  — rslidar_sdk v1.5.17 + ENABLE_IMU_DATA_PARSE=ON
发布: /imu/data (sensor_msgs/Imu)           — FAST-LIO2 期望的通用 IMU 话题

速腾 Airy 默认 IMU 参数:
  - 加速度量程: ±4g → 原始值×4/32768×9.80665 → m/s²
  - 陀螺仪量程: ±500dps → 原始值×500/32768×π/180 → rad/s
  (官方驱动已做转换, 此处直接透传)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuConverter(Node):
    def __init__(self):
        super().__init__('imu_converter')
        self.subscription = self.create_subscription(
            Imu, '/rslidar_imu', self.callback, 10)
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self._msg_count = 0
        self.get_logger().info("IMU relay started: /rslidar_imu_data -> /imu/data")

    def callback(self, msg: Imu):
        self._msg_count += 1
        # 确保 frame_id 与 URDF 中 imu_link 一致
        msg.header.frame_id = "imu_link"
        self.publisher.publish(msg)
        if self._msg_count % 200 == 1:  # 每 200 条(约 1 秒)打一次状态
            az = msg.linear_acceleration.z
            self.get_logger().info(
                f"IMU relay OK (cnt={self._msg_count}), "
                f"accel_z={az:.3f} m/s² "
                f"{'← 静止时接近 -9.8, 正常!' if -10.5 < az < -9.0 else ''}")


def main(args=None):
    rclpy.init(args=args)
    node = ImuConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
