#!/usr/bin/env python3
"""
全向靠泊作业节点 (Omni Docking Node)

基于激光雷达侧向测距的物理闭环控制，实现：
  DOCKING_IN  → 蟹行平移贴近鱼池
  FEEDING     → 停留投喂
  DOCKING_OUT → 蟹行退回通道，归还 Nav2 控制权

触发方式：向 /start_docking 发布 Bool(True)
状态反馈：/docking_status 发布 Bool(True) 表示靠泊周期完成
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time
import math


class OmniDockingNode(Node):
    def __init__(self):
        super().__init__('omni_docking_node')

        # 1. 声明工作参数
        self.declare_parameter('docking_speed', 0.15)       # 蟹行靠泊速度 (m/s)
        self.declare_parameter('target_distance', 0.2)      # 距离鱼池的最终目标距离 (m)
        self.declare_parameter('feeding_time', 5.0)         # 投喂停留时间 (秒)
        self.declare_parameter('pond_direction', 'left')    # 鱼池在车的哪边 ('left' 或 'right')

        self.dock_speed = self.get_parameter('docking_speed').value
        self.target_dist = self.get_parameter('target_distance').value
        self.feed_time = self.get_parameter('feeding_time').value
        self.pond_dir = self.get_parameter('pond_direction').value

        # 2. 话题发布与订阅
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(Bool, '/docking_status', 10)  # 告诉 Nav2 是否完成

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.trigger_sub = self.create_subscription(
            Bool, '/start_docking', self.trigger_callback, 10)

        # 3. 状态机变量
        # 状态: 'IDLE', 'DOCKING_IN', 'FEEDING', 'DOCKING_OUT'
        self.state = 'IDLE'
        self.current_lateral_dist = 999.0  # 当前侧向距离
        self.start_feed_time = 0.0         # 记录投喂开始的时间

        # 控制循环，10Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Omni Docking Node 已启动，等待靠泊指令...")

    # ------------------------------------------------------------------
    def trigger_callback(self, msg: Bool):
        """接收上层 (Nav2 或手动) 发来的启动信号"""
        if msg.data and self.state == 'IDLE':
            self.get_logger().info("收到靠泊指令！开始侧向蟹行靠泊...")
            self.state = 'DOCKING_IN'

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        """利用雷达测算侧向距离 (检测鱼池墙壁)"""
        # 雷达正前方 0°，左侧 +90°，右侧 -90°
        # 取侧面 ±5° 范围内的雷达点求平均，过滤噪点
        angle_range = math.radians(5.0)
        target_angle = math.pi / 2.0 if self.pond_dir == 'left' else -math.pi / 2.0

        valid_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle - target_angle) < angle_range:
                if msg.range_min < r < msg.range_max:
                    valid_ranges.append(r)

        if valid_ranges:
            self.current_lateral_dist = sum(valid_ranges) / len(valid_ranges)

    # ------------------------------------------------------------------
    def control_loop(self):
        """状态机主循环"""
        cmd = Twist()

        if self.state == 'IDLE':
            return  # 待命状态，什么都不发，让 Nav2 自由控制

        elif self.state == 'DOCKING_IN':
            # 正在靠拢
            if self.current_lateral_dist > self.target_dist:
                cmd.linear.y = (
                    self.dock_speed if self.pond_dir == 'left' else -self.dock_speed
                )
            else:
                # 距离达标，踩刹车，切换到投喂状态
                self.get_logger().info(
                    f"到达目标距离 ({self.target_dist}m)，停止移动，开始投喂。")
                cmd.linear.y = 0.0
                self.state = 'FEEDING'
                self.start_feed_time = time.time()

        elif self.state == 'FEEDING':
            # 投喂中，保持不动
            cmd.linear.y = 0.0
            if time.time() - self.start_feed_time > self.feed_time:
                self.get_logger().info("投喂完成！开始横移退出...")
                self.state = 'DOCKING_OUT'

        elif self.state == 'DOCKING_OUT':
            # 正在退出，回到大约 1.5m 的安全通道位置
            safe_distance = 1.5
            if self.current_lateral_dist < safe_distance:
                cmd.linear.y = (
                    -self.dock_speed if self.pond_dir == 'left' else self.dock_speed
                )
            else:
                self.get_logger().info(
                    "已安全退回通道，靠泊任务结束，归还底盘控制权。")
                cmd.linear.y = 0.0
                self.state = 'IDLE'

                # 发送完成信号给 Nav2，通知它可以去下一个点了
                done_msg = Bool()
                done_msg.data = True
                self.status_pub.publish(done_msg)

        # 发布速度指令 (纯 Vy 横移)
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = OmniDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("靠泊节点已终止。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
