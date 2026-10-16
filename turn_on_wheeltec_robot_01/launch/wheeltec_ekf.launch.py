from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def read_bool_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()

    if value not in ('true', 'false'):
        raise RuntimeError(
            f"Launch argument '{name}' must be 'true' or 'false', "
            f"but got: '{value}'"
        )

    return value == 'true'

def launch_setup(context):
    carto_slam = read_bool_argument(context, 'carto_slam')
    use_imu = read_bool_argument(context, 'use_imu')

    package_share = Path(
        get_package_share_directory('turn_on_wheeltec_robot_01')
    )
    config_dir = package_share / 'config'

    # 优先级：
    # 1. carto_slam=true -> ekf_carto.yaml
    # 2. carto_slam=false, use_imu=true -> ekf.yaml
    # 3. carto_slam=false, use_imu=false -> ekf_no_imu.yaml
    if carto_slam:
        config_file = config_dir / 'ekf_carto.yaml'
        mode = 'cartographer'
    elif use_imu:
        config_file = config_dir / 'ekf.yaml'
        mode = 'wheel odom + IMU'
    else:
        config_file = config_dir / 'ekf_no_imu.yaml'
        mode = 'wheel odom only'

    if not config_file.is_file():
        raise RuntimeError(f'EKF config file not found: {config_file}')

    return [
        LogInfo(
            msg=f'[wheeltec_ekf] mode={mode}, config={config_file}'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',

            # 必须与三个 YAML 的顶层名称一致
            name='ekf_filter_node',

            output='screen',
            parameters=[str(config_file)],

            # 当前项目对外使用 /odom_combined
            remappings=[
                ('/odometry/filtered', '/odom_combined'),
            ],
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'carto_slam',
            default_value='false',
            description='Use Cartographer-specific EKF configuration',
        ),

        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description=(
                'Use IMU in EKF when carto_slam=false. '
                'true=wheel odom + IMU, false=wheel odom only'
            ),
        ),

        OpaqueFunction(function=launch_setup),
    ])
