#!/bin/bash
# lslidar_motor_start.sh — 延迟 3 秒后向 2D 雷达发送电机启动命令
sleep 3
ros2 topic pub --once /x10/motor_control std_msgs/msg/Int8 "data: 1"
echo "[lslidar_motor_start] Motor control sent to LSLIDAR X10"
