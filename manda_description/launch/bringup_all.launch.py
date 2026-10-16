"""
bringup_all.launch.py — 一键总控点火文件

启动：
  1) 底盘 CAN 桥接 (manda_chassis_bridge)
  2) 2D LSLIDAR X10 雷达 (当前启用, /x10/lslidar_point_cloud + /scan)
  3) 机器人模型 + EKF 融合 (robot_core)
  4) Nav2 导航栈 (可选)

注：3D 速腾雷达 + FAST-LIO2 已注释，切换回来时取消注释即可。

用法：
  ros2 launch manda_description bringup_all.launch.py                     # 2D雷达(默认) + 底盘 + EKF
  ros2 launch manda_description bringup_all.launch.py nav:=true            # 含 Nav2 导航
  ros2 launch manda_description bringup_all.launch.py lidar_2d:=false      # 不启动 2D 雷达
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- 可选开关 ----
    nav_enabled = LaunchConfiguration("nav", default="false")
    slam_enabled = LaunchConfiguration("slam", default="false")     # SLAM 单独用 manda_slam_toolbox.launch.py 启动
    lidar_enabled = LaunchConfiguration("lidar", default="false")   # 3D 速腾雷达（已注释）
    lidar_2d_enabled = LaunchConfiguration("lidar_2d", default="true")  # 2D LSLIDAR X10（当前启用）
    imu_enabled = LaunchConfiguration("imu", default="true")        # 十轴 IMU（当前启用）
    map_path = LaunchConfiguration("map", default=os.path.join("/home/sunrise/manda/maps", "latest_map.yaml"))

    # 0. CAN 总线初始化 — 一键启动前自动配置 can0 接口
    can_init = ExecuteProcess(
        cmd=['sudo', 'bash', '-c',
             'modprobe can_raw 2>/dev/null; '
             'modprobe can 2>/dev/null; '
             'ip link set can0 down 2>/dev/null; '
             'ip link set can0 type can bitrate 500000; '
             'ip link set up can0 && echo "CAN0 UP: 500 kbps" || echo "CAN0 FAILED"'],
        name='can_init',
        output='screen',
        shell=False)

    # 1. 底盘 CAN 桥接 (manda_chassis_bridge) — 等待 CAN 初始化完成后再启动
    chassis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("manda_chassis_bridge"),
                "launch", "chassis.launch.py")))

    # 2. 机器人模型 + TF + EKF 融合
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("manda_description"),
                "launch", "robot_core.launch.py")))

    # 3. 2D LSLIDAR X10 雷达驱动（当前启用）
    lslidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lslidar_driver"),
                "launch", "lslidar_x10_launch.py")),
        condition=IfCondition(lidar_2d_enabled))

    # 3.1 2D 雷达电机自动启动（延迟 3 秒）
    lslidar_motor_start = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/x10/motor_control',
                     'std_msgs/msg/Int8', '{data: 1}'],
                output='screen')
        ],
        condition=IfCondition(lidar_2d_enabled))

    # 3.2 TF: laser_link → laser — 对齐 LSLIDAR X10 frame_id 与 URDF
    tf_laser_link_to_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_link_to_laser",
        arguments=["0", "0", "0", "0", "0", "0", "laser_link", "laser"],
        condition=IfCondition(lidar_2d_enabled))

    # 3.3 十轴 IMU (WitMotion) — 通过串口发布 /imu/data  使用 udev 固化端口
    imu_node = Node(
        package="wit_ros2_imu",
        executable="wit_ros2_imu",
        name="imu_driver",
        output="screen",
        condition=IfCondition(imu_enabled),
        parameters=[{"port": "/dev/imu_usb", "baud": 9600}],
        remappings=[("imu/data_raw", "imu/data")])

    # 4. 速腾激光雷达驱动 (rslidar_sdk) — 暂时注释，切换回 3D 时取消注释
    # rslidar_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("rslidar_sdk"),
    #             "launch", "start.py")),
    #     condition=IfCondition(lidar_enabled))

    # 5. FAST-LIO2 建图 — 暂时注释（2D 雷达用 slam_toolbox 代替）
    # fast_lio = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("fast_lio"),
    #             "launch", "mapping.launch.py")),
    #     launch_arguments={"config_file": "velodyne.yaml", "rviz": "false"}.items(),
    #     condition=IfCondition(lidar_enabled))

    # 6. IMU 转换器 — 暂时注释（2D 雷达不需要 IMU 转发）
    # imu_converter = Node(
    #     package="manda_description",
    #     executable="imu_converter.py",
    #     name="imu_converter",
    #     output="screen",
    #     condition=IfCondition(lidar_enabled))

    # ---- 以下静态 TF 已注释: 由 EKF/SLAM/URDF 动态接管 ----
    # # 7. TF 桥接：map ← camera_init (FAST-LIO2 原点)
    # tf_map_to_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_camera_init",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "camera_init"])

    # # 8. TF: body → base_footprint (已由 URDF + robot_state_publisher 发布)
    # tf_body_to_footprint = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="body_to_base_footprint",
    #     arguments=["0", "0", "0", "0", "0", "0", "body", "base_footprint"])

    # # 9. TF: map → odom (应由 SLAM (建图) 或 AMCL (导航) 动态发布)
    # tf_map_to_odom = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="map_to_odom",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"])

    # # 10. TF: odom → camera_init (视觉 SLAM 用)
    # tf_odom_to_camera = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="odom_to_camera_init",
    #     arguments=["0", "0", "0", "0", "0", "0", "odom", "camera_init"])

    # 11. Nav2 导航栈 (可选)
    nav2_pkg = get_package_share_directory("manda_description")
    nav2_share = get_package_share_directory("nav2_bringup")

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        condition=IfCondition(nav_enabled),
        parameters=[{
            "target_frame": "laser",
            "transform_tolerance": 0.01,
            "min_height": 0.0,
            "max_height": 2.0,
            "angle_min": -3.141592653589793,
            "angle_max": 3.141592653589793,
            "angle_increment": 0.0043633231,
            "scan_time": 0.1,
            "range_min": 0.15,
            "range_max": 20.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
        remappings=[("cloud_in", "/x10/lslidar_point_cloud"), ("scan", "/scan_filtered")],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")),
        condition=IfCondition(nav_enabled),
        launch_arguments={
            "map": map_path,
            "params_file": os.path.join(nav2_pkg, "config", "nav2_manda_2d.yaml"),
            "use_sim_time": "false",
            "slam": slam_enabled,
            "use_composition": "False",
        }.items(),
    )

    # 将底盘放在 can_init 之后启动（等待 CAN 就绪）
    chassis_delayed = TimerAction(
        period=2.0,
        actions=[chassis],
    )

    return LaunchDescription([
        DeclareLaunchArgument("nav", default_value="false",
                      description="启动 Nav2 导航栈 (建图时保持 false，导航时单独启动 bringup_navigation.launch.py)"),
        DeclareLaunchArgument("slam", default_value="True",
                              description="True=SLAM 建图, False=加载已有地图"),
        DeclareLaunchArgument("lidar", default_value="false",
                              description="启动 3D 速腾雷达 + Fast-LIO（当前已注释）"),
        DeclareLaunchArgument("lidar_2d", default_value="true",
                              description="启动 2D LSLIDAR X10 雷达（当前启用）"),
        DeclareLaunchArgument("imu", default_value="true",
                              description="启动十轴 IMU（当前启用）"),
        DeclareLaunchArgument("map", default_value=os.path.join("/home/sunrise/manda/maps", "latest_map.yaml"),
                              description="导航地图 yaml 路径"),
        can_init,          # 先配置 CAN 总线
        chassis_delayed,   # 等待 CAN 就绪后启动底盘桥接
        core,              # EKF + 模型
        lslidar_driver, lslidar_motor_start, tf_laser_link_to_laser, imu_node,
        # rslidar_driver,  # 3D 雷达 — 切换回来时取消注释
        # fast_lio,  # 3D Fast-LIO — 切换回来时取消注释
        # imu_converter,  # IMU 转发 — 切换回来时取消注释
        # tf_map_to_camera, # 注释: 视觉SLAM静态TF
        # tf_body_to_footprint, # 注释: URDF已发布
        # tf_map_to_odom, # 注释: 应由SLAM/AMCL动态发布
        # tf_odom_to_camera, # 注释: 视觉SLAM静态TF
        pointcloud_to_laserscan, nav2_bringup,
    ])
