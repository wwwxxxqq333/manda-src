"""
wit_imu.launch.py — 十轴 IMU 单独启动
发布 /imu/data_raw (sensor_msgs/Imu)
用法:
  ros2 launch wit_ros2_imu wit_imu.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wit_ros2_imu',
            executable='wit_ros2_imu',
            name='imu_driver',
            parameters=[{'port': '/dev/imu_usb'}, {'baud': 9600}, {'invert_yaw': False}],
            remappings=[('/wit/imu', '/imu/data_raw')],
            output='screen'
        ),
    ])
