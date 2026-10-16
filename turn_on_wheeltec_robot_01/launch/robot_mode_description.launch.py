import os, yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile


def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def spawn_robot_nodes(context, *args, **kwargs):
    """真正构造节点的函数，由 OpaqueFunction 调用"""
    # 1. 取得参数文件路径
    param_file = LaunchConfiguration('wheeltec_param_yaml').perform(context)
    robot_model_file = LaunchConfiguration('robot_model_yaml').perform(context)
    cfg = load_yaml(Path(param_file))
    model_cfg = load_yaml(Path(robot_model_file))

    # 2. 取车型
    car_mode = LaunchConfiguration('car_mode').perform(context) or cfg['car_mode']
    print(f'car_mode:{car_mode}')
    if car_mode not in model_cfg['robot_model']:
        raise ValueError(f'Unknown car_mode "{car_mode}" in {param_file}')

    model_cfg = model_cfg['robot_model'][car_mode]

    # 3. 构造节点
    # manda 车型使用 wheeltec_robot_urdf_01 包内的 manda.urdf，
    # 其余车型沿用原 wheeltec_robot_urdf 包的 {car_mode}_robot.urdf
    if car_mode == 'manda':
        urdf_pkg = 'wheeltec_robot_urdf_01'
        urdf_name = 'manda.urdf'
    else:
        urdf_pkg = 'wheeltec_robot_urdf'
        urdf_name = f'{car_mode}_robot.urdf'

    urdf_path = os.path.join(
        get_package_share_directory(urdf_pkg),
        'urdf',
        urdf_name
    )

    actions = [
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            arguments=[urdf_path],
        ),

        # base → link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_link',
            arguments=[*map(str, model_cfg['base_to_link']), 'base_footprint', 'base_link'],
        ),
        # base → radar
        # 注: manda 无相机/陀螺, base_to_camera 与 base_to_gyro 已移除
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar',
            arguments=[*map(str, model_cfg['base_to_radar']), 'base_footprint', 'radar'],
        ),
    ]

    # base → laser
    # manda 修复: 对 manda 车型不发布 base_footprint→laser 静态 TF。
    # 原因: manda.urdf 已通过 laser_link (laser_joint xyz=0.262,0,0.3716) 提供
    # base_footprint→laser_link 变换; 若再发布 base_footprint→laser 会造成同一 frame
    # 两条 TF 路径 → TF2 抖动 → 雷达点云乱窜。laser_link→laser 由建图入口
    # (online_async_launch.py) 的补丁 TF 负责。
    # 注: 实际雷达是 lslidar X10 (lidar_type=ls_M10_uart, 见 wheeltec_param.yaml),
    #     其 config/lslidar_x10.yaml 的 frame_id="laser_link",
    #     因此点云位置完全由 manda.urdf 的 laser_joint 决定 —— 改安装位置只改那里。
    if car_mode != 'manda':
        actions.insert(1,
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_laser',
                arguments=[*map(str, model_cfg['base_to_laser']), 'base_footprint', 'laser'],
            )
        )

    return actions


# ------------- Launch 描述 -------------
def generate_launch_description():
    return LaunchDescription([
        # 1. 声明参数
        DeclareLaunchArgument(
            'wheeltec_param_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot_01'),
                'config', 'wheeltec_param.yaml'),
            description='Path to wheeltec_param.yaml'
        ),
        DeclareLaunchArgument(
            'robot_model_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot_01'),
                'config', 'robot_model.yaml'),
            description='Path to robot_model.yaml'
        ),
        DeclareLaunchArgument(
            'car_mode',
            default_value='',
            description='Which robot mode to launch (mini_akm, mini_mec...)'
        ),

        # 2. 用 OpaqueFunction 在运行时解析 yaml 并构造节点
        OpaqueFunction(function=spawn_robot_nodes)
    ])
