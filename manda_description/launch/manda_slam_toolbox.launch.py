"""
manda_slam_toolbox.launch.py — 2D 在线 SLAM 建图

启动: slam_toolbox (异步模式), 纯建图, 不含底层驱动
输入: /scan (LaserScan) + TF odom->base_footprint (来自 EKF)
输出: /map (OccupancyGrid) + map->odom TF

用法:
  ros2 launch manda_description manda_slam_toolbox.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("manda_description")
    slam_config = os.path.join(pkg_share, "config", "mapper_params_online_async.yaml")

    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),
    ])
