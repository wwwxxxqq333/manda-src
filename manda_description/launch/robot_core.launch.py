"""
robot_core.launch.py — 实车「一键点火」总控文件

启动：
  1) robot_state_publisher  → 发布 TF 骨架 (URDF)
  2) ekf_node               → 融合 /odom_wheel + /imu/data → /odometry/filtered

依赖：
  - 必须先启动 CAN 底盘节点 (chassis_can_bridge)
  - 必须先启动速腾雷达 (rslidar_sdk)

用法：
  ros2 launch manda_description robot_core.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory("manda_description")
    # 部分 Humble 环境没有 xacro Python API，直接用预生成的 .urdf
    urdf_path = os.path.join(pkg_path, "urdf", "fwids_vehicle.urdf")

    with open(urdf_path, "r") as f:
        robot_description = {"robot_description": f.read()}

    ekf_config_path = os.path.join(pkg_path, "config", "ekf.yaml")

    return LaunchDescription(
        [
            # 1. 发布机器人 TF 骨架
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            # 2. 启动 EKF 里程计融合 (小脑)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config_path],
            ),
        ]
    )
