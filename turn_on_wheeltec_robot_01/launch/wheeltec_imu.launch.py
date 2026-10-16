import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    manda 新增 IMU: WitMotion 十轴 (wit_ros2_imu 包)
    启动后发布 /imu/data_raw (frame_id=imu_link, 与 manda.urdf 对齐)
    EKF 通过 imu0: /imu/data_raw 融合其 yaw / yaw_vel。
    """
    # 注意: wit_ros2_imu 的 setup.py 将 launch 文件直接安装到 share/wit_ros2_imu/
    # 根目录 (而非 launch/ 子目录), 因此这里直接拼接文件名。
    wit_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wit_ros2_imu'),
            'wit_imu.launch.py'
        ))
    )
    return LaunchDescription([wit_imu])
