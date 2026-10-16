雷达驱动
ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/home/sunrise/manda/src/rslidar-workspace/src/rslidar_sdk-v1.5.17/config/config.yaml
RViz2：
rviz2 -d /home/sunrise/manda/src/rslidar-workspace/src/rslidar_sdk-v1.5.17/rviz/rviz2.rviz
frame_id: rslidar







ros2 launch manda_description bringup_all.launch.py lidar:=True nav:=False
ros2 run rviz2 rviz2
