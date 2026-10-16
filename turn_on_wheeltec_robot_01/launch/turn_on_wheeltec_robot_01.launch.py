import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot_01')
    launch_dir = os.path.join(bringup_dir, 'launch')

    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam', default_value='false')

    # manda 修复: 透传 use_imu 开关 (隔离测试 IMU 是否导致雷达乱窜)
    use_imu = LaunchConfiguration('use_imu', default='true')
    use_imu_dec = DeclareLaunchArgument('use_imu', default_value='true')

    # CAN 底盘启动 (与原包的 base_serial.launch.py 对应，改用 CAN 协议)
    # 副本: 原 chassis_bridge.launch.py 重命名为 base_serial.launch.py
    can_chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
    )

    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
        launch_arguments={'carto_slam': carto_slam, 'use_imu': use_imu}.items(),
    )

    # manda 新增 WitMotion 十轴 IMU (独立节点, 发布 /imu/data_raw 供 EKF 融合)
    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_imu.launch.py')),
    )

    # 注意: manda.urdf 全部关节均为 fixed, 且 CAN 桥接节点已发布 /joint_states,
    # 因此不再需要 joint_state_publisher 节点 (原厂 mini 系车型才需要)
    car_mode_type = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
    )

    ld = LaunchDescription()
    ld.add_action(carto_slam_dec)
    ld.add_action(use_imu_dec)
    ld.add_action(can_chassis)
    ld.add_action(robot_ekf)
    ld.add_action(imu_driver)
    ld.add_action(car_mode_type)
    return ld
