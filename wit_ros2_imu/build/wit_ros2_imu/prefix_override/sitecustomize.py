import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/nvme/sunrise/manda_cpy/src/wit_ros2_imu/install/wit_ros2_imu'
