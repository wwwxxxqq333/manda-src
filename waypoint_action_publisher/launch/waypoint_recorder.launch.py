#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_recorder.launch.py — 位姿采点

启动 position_recorder，订阅定位话题，把当前底盘位姿追加保存为 JSON 航点。

用法:
  ros2 launch waypoint_action_publisher waypoint_recorder.launch.py
  ros2 launch waypoint_action_publisher waypoint_recorder.launch.py \\
      topic_type:=odom topic_name:=/odom

保存方式（任选）:
  1) ros2 service call /save_position std_srvs/srv/Trigger
  2) 在运行本 launch 的终端输入 r 并回车
  3) auto_save:=true auto_save_interval:=10.0

输出位置:
  默认写到节点内置目录（源码树 waypoints/current_position.json，构建时注入），
  与航点导航节点的默认读取位置一致。要改路径就显式传 save_directory。

前置条件:
  - 底盘 + EKF 已启动
  - 定位已启动（amcl_pose 模式下需要 AMCL 在跑，否则收不到 /amcl_pose）
  - ROS_DOMAIN_ID 与其他终端一致（本工程 bringup.sh 默认 42）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_node(context):
    params = {
        "topic_type": LaunchConfiguration("topic_type").perform(context),
        "topic_name": LaunchConfiguration("topic_name").perform(context),
        "output_file": LaunchConfiguration("output_file").perform(context),
        "auto_save": LaunchConfiguration(
            "auto_save").perform(context).lower() == "true",
        "auto_save_interval": float(
            LaunchConfiguration("auto_save_interval").perform(context)),
    }

    # 仅在显式指定时覆盖保存目录，否则沿用节点内置默认（源码树 waypoints/）
    save_directory = LaunchConfiguration("save_directory").perform(context).strip()
    if save_directory:
        params["save_directory"] = save_directory

    return [
        Node(
            package="waypoint_action_publisher",
            executable="position_recorder",
            name="position_recorder",
            output="screen",
            emulate_tty=True,      # 让"输入 r 保存"的终端交互可用
            parameters=[params],
        )
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "topic_type", default_value="amcl_pose",
            description="定位话题类型: amcl_pose / odom / pose_stamped"),
        DeclareLaunchArgument(
            "topic_name", default_value="/amcl_pose",
            description="定位话题名"),
        DeclareLaunchArgument(
            "output_file", default_value="current_position.json",
            description="输出文件名"),
        DeclareLaunchArgument(
            "save_directory", default_value="",
            description="输出目录（留空则用节点内置默认）"),
        DeclareLaunchArgument(
            "auto_save", default_value="false",
            description="是否定时自动保存"),
        DeclareLaunchArgument(
            "auto_save_interval", default_value="5.0",
            description="自动保存间隔（秒）"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_make_node)])
