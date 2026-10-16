import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- URDF file path ---------------------------------------------------
    urdf_path = os.path.join(
        get_package_share_directory("manda_description"),
        "urdf",
        "fwids_vehicle.urdf",
    )
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ---- launch arguments ---------------------------------------------------
    use_gui = LaunchConfiguration("use_gui", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    # ---- robot_state_publisher ---------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ---- joint_state_publisher_gui -----------------------------------------
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )

    # ---- rviz2 -------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("manda_description"),
        "config",
        "display.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Launch joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2",
            ),
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )
