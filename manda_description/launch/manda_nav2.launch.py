"""
manda_nav2.launch.py — 纯导航模式 (基于已保存的地图)

启动:
  1) map_server — 加载预存地图 (.yaml + .pgm)
  2) AMCL — 2D 自适应蒙特卡洛定位 (仅用 /scan + /odom)
  3) Nav2 bringup — 全局/局部规划器 + 控制器 (无 SLAM)

无 IMU 依赖，纯雷达+里程计导航

用法:
  ros2 launch manda_description manda_nav2.launch.py
  ros2 launch manda_description manda_nav2.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 参数 ----
    map_path = LaunchConfiguration("map", default="/home/sunrise/manda/maps/manda_map.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pkg_share = get_package_share_directory("manda_description")
    nav2_share = get_package_share_directory("nav2_bringup")

    # 1. Nav2 导航栈 (不含 SLAM)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "map": map_path,
            "params_file": os.path.join(pkg_share, "config", "nav2_manda_2d.yaml"),
            "use_sim_time": use_sim_time,
            "slam": "False",
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="/home/sunrise/manda/maps/manda_map.yaml",
                              description="预存地图 .yaml 路径"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="使用仿真时钟"),
        nav2_bringup,
    ])
