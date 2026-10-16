#ros2 run nav2_map_server map_saver_cli -f ~/map
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions


def generate_launch_description():

    # 保存地图到当前 wheeltec_nav2 包目录下的 map 文件夹（文件名固定为 WHEELTEC）
    # 用 get_package_share_directory 动态解析, 兼容 /home 与 /nvme 等多份工作区
    nav2_share_dir = get_package_share_directory('wheeltec_nav2')
    # install 为 symlink-install 时 share/map 即源码 src/wheeltec_robot_nav2/map
    map_dir = os.path.join(nav2_share_dir, 'map')
    map_path = os.path.join(map_dir, 'WHEELTEC')

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_path],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)
    return ld
 
