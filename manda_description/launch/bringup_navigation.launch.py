#!/usr/bin/env python3
"""
bringup_navigation.launch.py — 纯导航一键启动（地图加载 + 定位 + 规划 + 控制）

===== 设计原则 =====
  底层硬件（底盘、雷达、EKF、robot_state_publisher）由 bringup_all.launch.py 负责。
  本文件只负责上层导航算法，不包含任何 URDF/TF 骨架代码，避免节点冲突。

===== 启动的组件 =====
    nav2_bringup (slam:=False) 内部自动启动:
        ├── map_server                 — 加载静态 2D 地图，发布 /map
        ├── amcl                       — 蒙特卡洛定位，发布 map → odom TF
        ├── lifecycle_manager_localization — 激活 map_server + amcl 生命周期
    ├── planner_server             — 全局路径规划 (SmacHybrid)
    ├── controller_server          — 局部路径跟踪 (RegulatedPurePursuit)
    ├── behavior_server            — 行为树引擎
    ├── bt_navigator               — 导航行为树
    ├── waypoint_follower          — 航点跟随
    ├── global_costmap / local_costmap — 全局/局部代价地图
    └── lifecycle_manager_navigation   — 激活导航节点生命周期
  本文件额外启动:
    └── rviz2 (可选)               — 加载 Nav2 默认 RViz 配置

===== 用法 =====
  # 标准导航 (需要先启动底层)
  # 终端 1: 底层硬件
  ros2 launch manda_description bringup_all.launch.py lidar_2d:=true imu:=true nav:=False
  # 终端 2: 纯导航
  ros2 launch manda_description bringup_navigation.launch.py

  # 覆盖地图路径
  ros2 launch manda_description bringup_navigation.launch.py map:=/path/to/map.yaml

  # 不启动 RViz (无显示器/远程SSH)
  ros2 launch manda_description bringup_navigation.launch.py use_rviz:=false

===== 依赖前提 =====
  必须先启动 bringup_all.launch.py (nav:=False)，确保以下内容已在运行:
    - CAN 底盘桥接 (发布 /odom_wheel)
    - EKF 里程计融合 (发布 odom → base_footprint TF + /odometry/filtered)
    - robot_state_publisher (发布 URDF 静态 TF)
    - 2D/3D 雷达驱动 (发布 /scan 或 /scan_filtered)
    - IMU 驱动 (发布 /imu/data)

===== 参考来源 =====
  - Nav2 官方 nav2_bringup/launch/bringup_launch.py
  - wheeltec turn_on_wheeltec_robot 导航架构
  - manda_description 已有 bringup_all.launch.py / manda_nav2.launch.py
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
    # ================================================================
    # 0. 包路径
    # ================================================================
    pkg_share = get_package_share_directory("manda_description")
    nav2_share = get_package_share_directory("nav2_bringup")
    # ================================================================
    # 1. 可覆盖的 Launch 参数
    #    格式: ros2 launch ... <参数名>:=<值>
    # ================================================================

    # 静态地图路径 (必须对应已有的 .yaml + .pgm 文件)
    map_path = LaunchConfiguration(
        "map",
        default="/home/sunrise/manda/maps/manda_map.yaml",
    )

    # Nav2 全部参数文件 (包含 AMCL、planner、controller、costmap 等所有配置)
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(pkg_share, "config", "nav2_manda_2d.yaml"),
    )

    # RViz2 配置文件 (默认使用 Nav2 官方的完整导航视图)
    rviz_config = LaunchConfiguration(
        "rviz_config",
        default=os.path.join(nav2_share, "rviz", "nav2_default_view.rviz"),
    )

    # ---- 开关 ----
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    # ================================================================
    # 2. Nav2 Bringup — 导航全家桶
    #
    #    slam:=False 时由 Nav2 自己启动 localization + navigation，
    #    这里只负责传入地图和参数文件，不再额外手动起 map_server / amcl。
    # ================================================================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments=[
            ("map", map_path),
            ("params_file", params_file),
            ("use_sim_time", use_sim_time),
            ("slam", "False"),
            ("use_composition", "False"),
            ("autostart", "True"),
        ],
    )

    # ================================================================
    # 3. RViz2 (可选)
    # ================================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # ================================================================
    # 4. 组装
    # ================================================================
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="/home/sunrise/manda/maps/manda_map.yaml",
                description="静态地图 .yaml 路径 (对应的 .pgm 需在同一目录)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    pkg_share, "config", "nav2_manda_2d.yaml"
                ),
                description="Nav2 全部参数文件路径",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(
                    nav2_share, "rviz", "nav2_default_view.rviz"
                ),
                description="RViz2 配置文件路径",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="仿真时钟开关 (实车=false, Gazebo=true)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz2 可视化",
            ),
            nav2_bringup,
            rviz_node,
        ]
    )

