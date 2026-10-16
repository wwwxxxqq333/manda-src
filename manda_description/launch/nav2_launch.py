import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("manda_description")
    nav2_share = get_package_share_directory("nav2_bringup")

    params_file = os.path.join(pkg_share, "config", "nav2_manda_3d.yaml")
    default_map = os.path.join("/home/sunrise/manda/maps", "latest_map.yaml")
    map_file = LaunchConfiguration("map", default=default_map)
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    use_slam = LaunchConfiguration("slam", default="True")

    # 将 /rslidar_points 转为 Nav2/AMCL 需要的 LaserScan
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[{
            "target_frame": "rslidar",
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
        remappings=[("cloud_in", "/rslidar_points"), ("scan", "/scan_filtered")],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
            "use_sim_time": "false",
            "slam": use_slam,
            "use_composition": "False",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(
            nav2_share, "rviz", "nav2_default_view.rviz")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )y

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map", default_value=default_map,
                description="Path to map yaml"),
            DeclareLaunchArgument(
                "use_rviz", default_value="true",
                description="Launch RViz2"),
            DeclareLaunchArgument(
                "slam", default_value="True",
                description="Whether to run SLAM (True) or use a pre-built map (False)"),
            pointcloud_to_laserscan,
            nav2_bringup,
            rviz,
        ]
    )
