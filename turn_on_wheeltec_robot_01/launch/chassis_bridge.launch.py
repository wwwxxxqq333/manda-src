import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('turn_on_wheeltec_robot_01')

    # 默认参数文件
    default_yaml = os.path.join(pkg_share, 'config', 'manda_param.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_yaml,
        description='底盘参数 yaml 文件路径 (不同车型使用不同 yaml)',
    )

    param_file = LaunchConfiguration('param_file')

    chassis_node = Node(
        package='turn_on_wheeltec_robot_01',
        executable='chassis_bridge_node',
        name='chassis_bridge_node',
        output='screen',
        parameters=[param_file],
    )

    return LaunchDescription([
        param_file_arg,
        chassis_node,
    ])
