from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot_01')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot_01.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_lidar.launch.py')),
    )

    # 雷达 TF 对齐: lslidar 驱动 frame_id=laser_link, 而 base_to_laser 发布 base_footprint->laser。
    # 补齐 laser_link->laser (零偏移), 使 slam_toolbox 能通过 /scan 的 laser_link
    # 一路变换到 base_footprint/odom_combined, 否则雷达会乱窜/重影。
    tf_laser_link_to_laser = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_link_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'laser'],
    )

    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar, tf_laser_link_to_laser,
        launch_ros.actions.Node(
        	parameters=[
        		get_package_share_directory("wheeltec_slam_toolbox") + '/config/mapper_params_online_async.yaml'
        	],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            remappings=[('odom','odom_combined')]
        )
    ])
