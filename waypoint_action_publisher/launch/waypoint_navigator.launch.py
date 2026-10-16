#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_navigator.launch.py — JSON 航点导航

启动 json_waypoint_navigator，读取 JSON 航点文件并逐个通过
Nav2 的 NavigateToPose 动作导航到位，带 TF 到点精度校验与可选精修。

用法:
  # 用节点内置默认航点文件（源码树 waypoints/current_position.json，构建时注入）
  ros2 launch waypoint_action_publisher waypoint_navigator.launch.py

  # 指定航点文件
  ros2 launch waypoint_action_publisher waypoint_navigator.launch.py \\
      waypoints_file:=/abs/path/current_position.json

  # 调整到点阈值（默认与 nav2_manda_2d.yaml 对齐）
  ros2 launch waypoint_action_publisher waypoint_navigator.launch.py \\
      yaw_tolerance_rad:=0.60 position_tolerance_m:=0.30

前置条件:
  - ./bringup.sh nav 已启动（底盘 + 雷达 + EKF + Nav2，含 AMCL 定位）
  - ROS_DOMAIN_ID 与其他终端一致（本工程 bringup.sh 默认 42）

节点执行完所有航点后自动退出；任一航点失败则终止剩余航点并以非 0 退出。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    params = {
        "frame_id": LaunchConfiguration("frame_id").perform(context),
        "robot_base_frame": LaunchConfiguration("robot_base_frame").perform(context),
        "position_tolerance_m": float(
            LaunchConfiguration("position_tolerance_m").perform(context)),
        "yaw_tolerance_rad": float(
            LaunchConfiguration("yaw_tolerance_rad").perform(context)),
        "enable_pose_refine": LaunchConfiguration(
            "enable_pose_refine").perform(context).lower() == "true",
        "refine_max_attempts": int(
            LaunchConfiguration("refine_max_attempts").perform(context)),
        "refine_timeout_sec": int(
            LaunchConfiguration("refine_timeout_sec").perform(context)),
        "nav_timeout_sec": int(LaunchConfiguration("nav_timeout_sec").perform(context)),
        "wait_between_points": LaunchConfiguration(
            "wait_between_points").perform(context).lower() == "true",
    }

    # 仅在显式指定时覆盖航点文件路径，否则沿用节点内置默认（源码树 waypoints/）
    waypoints_file = LaunchConfiguration("waypoints_file").perform(context).strip()
    if waypoints_file:
        params["waypoints_file"] = waypoints_file

    return [
        Node(
            package="waypoint_action_publisher",
            executable="json_waypoint_navigator",
            name="json_waypoint_navigator",
            output="screen",
            parameters=[params],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "waypoints_file", default_value="",
            description="航点 JSON 文件绝对路径（留空则用节点内置默认）"),
        DeclareLaunchArgument(
            "frame_id", default_value="map",
            description="航点目标所在的全局坐标系"),
        DeclareLaunchArgument(
            "robot_base_frame", default_value="base_footprint",
            description="机器人基座坐标系（本工程为 base_footprint，非 base_link）"),
        DeclareLaunchArgument(
            "position_tolerance_m", default_value="0.30",
            description="到点距离阈值（建议与 Nav2 xy_goal_tolerance 一致）"),
        DeclareLaunchArgument(
            "yaw_tolerance_rad", default_value="0.60",
            description="到点角度阈值（建议与 Nav2 yaw_goal_tolerance 一致）"),
        DeclareLaunchArgument(
            "enable_pose_refine", default_value="true",
            description="是否允许到点精修（仍受每个航点的 enable_refine 控制）"),
        DeclareLaunchArgument(
            "refine_max_attempts", default_value="2",
            description="单点最大精修次数"),
        DeclareLaunchArgument(
            "refine_timeout_sec", default_value="30",
            description="单次精修超时（秒）"),
        DeclareLaunchArgument(
            "nav_timeout_sec", default_value="180",
            description="单个航点导航超时（秒）"),
        DeclareLaunchArgument(
            "wait_between_points", default_value="true",
            description="是否在航点间按 wait_time 等待"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_make_node)])
