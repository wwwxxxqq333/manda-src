import time
import math
import os
import termios
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


# 定义IMU驱动节点类
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(list_data, check_data):
    return sum(list_data) & 0xff == check_data


def handle_serial_data(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag
    # manda V2.03: 只在实际收到 0x52 (陀螺角速度) 帧时才发布 IMU 消息。
    #   旧实现收到 0x51/0x52/0x53 任一帧都发布 → 同一陀螺角速度值被重复发布、
    #   带新时间戳, 导致 EKF 对 yaw_vel 重复加权/时间对齐错误。
    #   0x51(加速度)/0x53(姿态角)只更新缓存, 不触发发布。
    publish_now = False
    buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:
        return
    else:
        data_buff = list(buff.values())
        if buff[1] == 0x51:
            if check_sum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                # 只更新加速度缓存, 不发布
        elif buff[1] == 0x52:
            if check_sum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
                # 只有真实角速度更新后才发布
                publish_now = True
        elif buff[1] == 0x53:
            if check_sum(data_buff[0:10], data_buff[10]):
                angle_degree = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 180 for i in range(0, 3)]
                # 只更新姿态角缓存, 不发布
        elif buff[1] == 0x54:
            if check_sum(data_buff[0:10], data_buff[10]):
                magnetometer = hex_to_short(data_buff[2:10])
        else:
            buff = {}
            key = 0

        buff = {}
        key = 0
        return publish_now
        # if angle_flag:
        #     stamp = rospy.get_rostime()
        #
        #     imu_msg.header.stamp = stamp
        #     imu_msg.header.frame_id = "base_link"
        #
        #     mag_msg.header.stamp = stamp
        #     mag_msg.header.frame_id = "base_link"
        #
        #     angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        #     qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])
        #
        #     imu_msg.orientation.x = qua[0]
        #     imu_msg.orientation.y = qua[1]
        #     imu_msg.orientation.z = qua[2]
        #     imu_msg.orientation.w = qua[3]
        #
        #     imu_msg.angular_velocity.x = angularVelocity[0]
        #     imu_msg.angular_velocity.y = angularVelocity[1]
        #     imu_msg.angular_velocity.z = angularVelocity[2]
        #
        #     imu_msg.linear_acceleration.x = acceleration[0]
        #     imu_msg.linear_acceleration.y = acceleration[1]
        #     imu_msg.linear_acceleration.z = acceleration[2]
        #
        #     mag_msg.magnetic_field.x = magnetometer[0]
        #     mag_msg.magnetic_field.y = magnetometer[1]
        #     mag_msg.magnetic_field.z = magnetometer[2]
        #
        #     imu_pub.publish(imu_msg)
        #     mag_pub.publish(mag_msg)


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        # 从 launch 参数中读取 port 和 baud
        self.declare_parameter('port', '/dev/imu_usb')
        self.declare_parameter('baud', 9600)
        # manda: yaw 极性开关 (V2.03 实车左转标定修正)
        #   实测 base_footprint→imu_link TF 的 RPY=(0,0,0) 无翻转,
        #   且左转时 odom_wheel.vz>0 而旧代码取反后 imu.wz<0 → 多取反了一次。
        #   因此默认 false = 不取反 (原厂方向), 车左转(逆时针)时 wz 应为正。
        #   true  = 取反 (仅当 IMU 物理装反时使用)
        self.declare_parameter('invert_yaw', False)
        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud').get_parameter_value().integer_value
        self.invert_yaw_ = self.get_parameter('invert_yaw').get_parameter_value().bool_value

        self.get_logger().info(
            f"IMU 配置: port={port_name}, baud={baud_rate}, "
            f"invert_yaw={self.invert_yaw_}"
        )

        # 初始化IMU消息
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'
        # manda 修复: 填充协方差, 否则 robot_localization EKF 会把 IMU 数据当"绝对准确",
        # 对陀螺零偏/噪声极度敏感 → 航向震荡 → 雷达点云乱窜。
        # 3x3 对角线: [x, y, z] 角速度协方差 (rad^2/s^2)
        self.imu_msg.angular_velocity_covariance[0] = 1.0e-3   # wx
        self.imu_msg.angular_velocity_covariance[4] = 1.0e-3   # wy
        self.imu_msg.angular_velocity_covariance[8] = 1.0e-3   # wz (yaw_vel)
        # 姿态/加速度协方差: EKF 配置中不融合这些量, 置为 -1 表示无效
        for i in range(9):
            self.imu_msg.orientation_covariance[i] = -1.0
            self.imu_msg.linear_acceleration_covariance[i] = -1.0

        # ============================================================
        # manda V2.13: z 轴陀螺零偏启动静止标定 + 静止死区
        # ------------------------------------------------------------
        # 实测(log/live_diag/ros_raw.csv)静止时 imu_wz 恒为 +0.0075 rad/s
        # (≈0.43 deg/s), 远大于 1 LSB(0.061 deg/s), 是真实零偏而非量化噪声。
        # EKF 只融合 yaw_vel(无绝对航向参考), 该零偏被直接积分 ->
        #   60s 累积 ~26deg, 300s 累积 ~129deg -> 观感就是"停一会儿点云自己漂"。
        # 对策:
        #   1. 开机时(车静止)采样 N 帧取均值作为零偏, 之后逐帧扣除
        #   2. 残差再用静止死区(2 LSB ≈ 0.002 rad/s)压成 0
        #   3. 标定期间对 EKF 发布零角速度, 避免把未扣除的零偏先积进去
        # ⚠ 启动后前约 8 秒必须保持车辆完全静止, 否则零偏会被标错
        # ============================================================
        self.bias_n_samples_ = 50      # 目标采样帧数
        self.bias_min_samples_ = 10    # 至少多少帧才认可用作标定
        self.bias_timeout_ = 8.0       # s: 超出此时间即使帧数不足也结束
        self.gyro_deadband_ = 0.002    # rad/s: 扣零偏后的静止死区
        self.bias_samples_ = []        # 已采样的 raw wz
        self.bias_t_start_ = None      # 标定开始时刻
        self.bias_z_ = None            # None = 尚未标定完成

        # 创建IMU数据发布器
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # 启动IMU驱动线程
        self.driver_thread = threading.Thread(target=self.driver_loop, args=(port_name, baud_rate))
        self.driver_thread.start()

    def driver_loop(self, port_name, baud_rate):
        # 打开串口 - 使用 os.open 而非 pyserial 以避开 CP2102 兼容性 bug
        try:
            # 解决 pyserial + CP2102 的 "device reports readiness to read but returned no data" 问题
            # 方法：直接用 os.open + termios 配置串口，然后循环用 os.read 读取
            self.fd = os.open(port_name, os.O_RDWR | os.O_NOCTTY)
            # 配置串口参数
            attrs = termios.tcgetattr(self.fd)
            # 设置波特率
            baud_map = {9600: termios.B9600, 115200: termios.B115200, 460800: termios.B460800}
            baud_const = baud_map.get(baud_rate, termios.B9600)
            attrs[4] = baud_const  # ispeed
            attrs[5] = baud_const  # ospeed

            # ============================================================
            # manda V2.13: 修正 termios 各位写错数组下标的问题
            # ------------------------------------------------------------
            # termios.tcgetattr 返回:
            #   [0]=iflag  [1]=oflag  [2]=cflag  [3]=lflag  [4]=ispeed  [5]=ospeed  [6]=cc
            # 旧代码把 CSIZE/CS8/PARENB/CSTOPB/CRTSCTS 以及 CREAD/CLOCAL 全部
            # 写进了 attrs[3](lflag) —— 而这些位都属于 cflag(attrs[2])。后果:
            #   ① cflag 中 CRTSCTS 从未被清除 -> 硬件流控可能仍开着(读阻塞风险)
            #   ② CREAD 没设进 cflag -> 接收器可能未使能
            #   ③ lflag 里塞了 cflag 位 -> 行规范(ICANON/ECHO)其实没被关掉
            # 现按正确下标重写:
            # ============================================================
            # --- iflag (1): 关掉所有输入变换 ---
            attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                          termios.ISTRIP | termios.INLCR | termios.IGNCR |
                          termios.ICRNL | termios.IXON)
            # --- oflag (1): 关掉输出处理 ---
            attrs[1] &= ~termios.OPOST
            # --- cflag (2): 8N1 + 无流控 + 使能接收 ---
            attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB |
                          termios.CRTSCTS)
            attrs[2] |= termios.CS8
            attrs[2] |= (termios.CREAD | termios.CLOCAL)
            # --- lflag (3): 彻底原始模式, 关行规范/回显/信号 ---
            attrs[3] = 0
            # 读超时 (VTIME=0, VMIN=1 = 阻塞读直到至少1字节)
            attrs[6][termios.VTIME] = 0
            attrs[6][termios.VMIN] = 1
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            self.get_logger().info("\033[32mSerial port opened successfully (native fd mode)...\033[0m")
        except Exception as e:
            self.get_logger().error(f"Serial port opening failure: {e}")
            exit(0)

        # 循环读取IMU数据 - 使用 os.read 逐字节读取
        buffer = bytearray()
        while True:
            try:
                # 读取一块数据
                chunk = os.read(self.fd, 256)
                if not chunk:
                    time.sleep(0.01)
                    continue
                # 逐字节处理
                for b in chunk:
                    tag = handle_serial_data(b)
                    if tag:
                        self.imu_data()
            except Exception as e:
                self.get_logger().error(f"imu read error: {e}")
                time.sleep(0.1)

    def imu_data(self):
        # handle_serial_data() 已经将 acceleration 转为 m/s²，angularVelocity 转为 rad/s
        # 这里直接使用，不做二次缩放
        accel_x, accel_y, accel_z = float(acceleration[0]), float(acceleration[1]), float(acceleration[2])
        gyro_x, gyro_y, gyro_z   = float(angularVelocity[0]), float(angularVelocity[1]), float(angularVelocity[2])

        # yaw 极性: 由 invert_yaw 参数控制 (V1.06 修正为取反, 可实车验证切换)
        yaw_sign = -1.0 if self.invert_yaw_ else 1.0
        wz_raw = yaw_sign * gyro_z

        # ============================================================
        # manda V2.13: 零偏标定 + 死区 (状态机)
        # ------------------------------------------------------------
        # 阶段A 采集中: 累计 wz_raw, 对外发布 0 (避免未扣零偏被 EKF 积进去)
        # 阶段B 已标定: wz = wz_raw - bias, 再经死区
        # ============================================================
        now_s = time.time()
        if self.bias_z_ is None:
            # ---- 阶段A: 采样 ----
            if self.bias_t_start_ is None:
                self.bias_t_start_ = now_s
                self.get_logger().info(
                    "\033[33m开始 z 轴陀螺零偏标定，请保持车辆完全静止\033[0m"
                )
            self.bias_samples_.append(wz_raw)
            elapsed = now_s - self.bias_t_start_
            enough = len(self.bias_samples_) >= self.bias_n_samples_
            timed_out = elapsed >= self.bias_timeout_
            if enough or (timed_out and len(self.bias_samples_) >= self.bias_min_samples_):
                self.bias_z_ = sum(self.bias_samples_) / len(self.bias_samples_)
                self.get_logger().info(
                    f"\033[32mz 轴陀螺零偏标定完成: bias={self.bias_z_:+.6f} rad/s "
                    f"({self.bias_z_ * 180.0 / math.pi:+.4f} deg/s), "
                    f"样本={len(self.bias_samples_)}, 耗时={elapsed:.2f}s\033[0m"
                )
                self.bias_samples_ = []
            # 标定期间对外发 0 角速度
            wz_out = 0.0
        else:
            # ---- 阶段B: 扣零偏 + 死区 ----
            wz_out = wz_raw - self.bias_z_
            if abs(wz_out) < self.gyro_deadband_:
                wz_out = 0.0

        # 更新IMU消息
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = wz_out

        angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
        # yaw 极性: 与 yaw_vel 保持一致
        angle_radian[2] = yaw_sign * angle_radian[2]

        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]

        # 发布IMU消息
        self.imu_pub.publish(self.imu_msg)

    def compute_orientation(self, wx, wy, wz, ax, ay, az, dt):
        # 计算旋转矩阵
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(ax), -math.sin(ax)],
                       [0, math.sin(ax), math.cos(ax)]])
        Ry = np.array([[math.cos(ay), 0, math.sin(ay)],
                       [0, 1, 0],
                       [-math.sin(ay), 0, math.cos(ay)]])
        Rz = np.array([[math.cos(wz), -math.sin(wz), 0],
                       [math.sin(wz), math.cos(wz), 0],
                       [0, 0, 1]])
        R = Rz.dot(Ry).dot(Rx)

        # 计算欧拉角
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2))
        yaw = math.atan2(R[1][0], R[0][0])

        return roll, pitch, yaw


def main():
    # 初始化ROS 2节点
    rclpy.init()
    node = IMUDriverNode()

    # 运行ROS 2节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # 停止ROS 2节点
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
