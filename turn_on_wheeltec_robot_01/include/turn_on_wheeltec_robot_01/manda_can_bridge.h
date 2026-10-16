#ifndef __MANDA_CAN_BRIDGE_H_
#define __MANDA_CAN_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <thread>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <stdexcept>   // manda V2.11: 单实例冲突时抛出明确异常

// ============================================================
// 四轮独立驱动与转向 (4WID-4WIS) 整车阿克曼 CAN 桥接
// ============================================================
// manda V2.19 架构 (官方 CAN 协议 V1.24) —— 【阿克曼 + 原地旋转】
// ------------------------------------------------------------
// 本车支持两种运动方式: 0x01 前后轮反向转向阿克曼，0x04 原地旋转。
//   正常运动 : VCU 自行解算四轮转角 (内轮更大/外轮更小) 与四轮轮速
//              (内慢外快)。上位机只下发整车速度, 不逐轮计算。
//   下发方式 : 0x402 的 Vx / Vy(恒 0) / Vz     (DLC=8, int16 × 1000)
//   协议帧   : 上位机 → 底盘: 0x401(模式切换, 只发 0x01), 0x402(整车速度)
//              底盘 → 上位机: 0x100(状态/电池/控制权), 0x101(轮速反馈),
//                             0x102(转角反馈), 0x103(故障), 0x104(运动反馈)
//   关键约束 : 切换 0x401 前后必须静默 >= 200ms, 否则切换会失败。
//
// 【原地旋转 (0x04)】
// ------------------------------------------------------------
// 为何删: 该车底盘 VCU 固件不会因 "Vx≈0 且 Vz≠0" 进入 0x04 自旋
//   (实测抓包: 下发纯旋转后 0x104 mode 长期停在 0x01, 车原地不动)。
//   而 0x01 阿克曼在 Vx=0 时物理上无法旋转 —— 保留 0x04 通路只会
//   让上游以为"在转"、实际不动, 反而卡死导航。
// 桥只负责执行上层命令，不决定何时旋转、旋转多少度或是否重规划；
// 这些决策由 Nav2 行为树的 Spin 恢复动作和 RotationShim 完成。
//
// 【唯一的运动约束 (阿克曼)】
// ------------------------------------------------------------
//   设 v_min = 最低行驶速度 (min_moving_speed_)
//     |Vx| <  v_min   →  Vx = 0, Vz = 0   (不允许"只转不走")
//     |Vx| >= v_min   →  |Vz| <= |Vx| / R_min   (走圆弧)
//   注意: **绝不能删除 Vz** —— 否则阿克曼无法拐弯。
//         被禁止的只是 "Vx=0 且 Vz≠0" 这一种组合 (即原地旋转)。
//   本约束在两处强制执行 (双重保险, 同一实现):
//     ① cmdVelCallback: 网络入口整形
//     ② sendVelocityCommand: CAN 最终出口 (拦速度斜坡产生的过渡帧)
//
// 旧 0x08 实现 (computeAckermannKinematics / 先转向后驱动斜坡 / DriveState)
// 保留在 .cpp 末尾作为回退参考, 正常运行路径不再调用。
// ============================================================

namespace manda_mode {
// 0x401 Byte[0] 与 0x104 Byte[6] 共用同一套运动模式编码。
// 本车实际使用两个: 0x01 (行驶, 默认) 与 0x04 (终端原地自旋)。
constexpr uint8_t FRONT_REAR_ACKERMANN = 0x01;  // 前后轮反向转向阿克曼 (默认/唯一行驶模式)
constexpr uint8_t CRAB                 = 0x02;  // 四轮同向斜移 (本车暂不用, VCU 支持)
// ★ 2026-09-28 恢复: VCU 实测支持 0x04 (遥控器可切入), 仅由终端航向校正触发。
//   0x04 下 VCU 只认 0x402 的 Vz, 自动摆四轮实现原地自旋。
constexpr uint8_t SPIN                 = 0x04;  // 原地自旋 (仅终端航向校正触发)
constexpr uint8_t INDIVIDUAL           = 0x08;  // 用户逐轮控制 (仅 LEGACY 参考)
constexpr uint8_t PARK                 = 0x10;  // 驻车

// 判断某模式是否为"控制器主动请求"的有效模式。
// 0x02/0x08/0x10 等仍按异常处理 —— 它们只可能来自遥控器残留。
inline bool isRequestedMode(uint8_t m) {
    return m == FRONT_REAR_ACKERMANN || m == SPIN;
}
}  // namespace manda_mode

class MandaCanBridge : public rclcpp::Node {
public:
    MandaCanBridge();
    ~MandaCanBridge();

private:
    // ============================================================
    // 底盘反馈状态聚合
    // ============================================================
    struct ChassisState {
        // 0x100 系统状态
        uint8_t  battery_alarm   = 0;
        uint8_t  control_mode    = 0;
        uint8_t  peripheral_bits = 0;
        uint8_t  battery_soc     = 0;
        int16_t  battery_voltage_10mv  = 0;
        int16_t  battery_current_10ma  = 0;
        // 0x101 四轮速度反馈 (m/s)
        float wheel_speed[4] = {0};
        // 0x102 四轮转向角度反馈 (度)
        float steer_angle[4] = {0};
        // 0x103 故障
        uint8_t  system_fault          = 0;
        uint8_t  drv_fault_walk_bits   = 0;
        uint8_t  drv_fault_steer_bits  = 0;
        uint8_t  drv_comm_lost_walk    = 0;
        uint8_t  drv_comm_lost_steer   = 0;
        // 0x104 运动反馈
        float    motion_vx = 0, motion_vy = 0, motion_vz = 0;
        uint8_t  current_mode   = 0;    // manda_mode::* (底盘实际运动模式)
        uint8_t  mode_switch_state = 1; // 0x00=切换完成, 0x01=切换中
    };

    // ---- 机器人几何参数 (仅 URDF/joint_states 与日志使用) ----
    double wheel_base_x_ = 0.24;  // 前后半轴距 (m)
    double wheel_base_y_ = 0.19;  // 左右半轮距 (m)
    double wheel_radius_ = 0.10;   // 车轮半径 (m)
    double wheel_base_  = 0.48;   // 全轴距 L = 2 * wheel_base_x_ (仅旧 0x08 用)
    double track_width_ = 0.38;   // 全轮距 W = 2 * wheel_base_y_ (仅旧 0x08 用)
    double max_steer_angle_ = 35.0; // 度: 旧 0x08 转向角安全限位 (0x01 由 VCU 自行约束)
    double max_wheel_speed_ = 2.0;  // m/s: 旧 0x08 单轮速度上限 (仅参考实现使用)

    // ---- 整车运动限制 (0x01 模式下由上位机保证) ----
    // 0x01 模式的四轮解算已交给 VCU, 上位机只需限制整车速度、加速度与曲率:
    //   设 v_min = min_moving_speed_ (最低行驶速度)
    //     |Vx| <  v_min  →  Vx = 0, Vz = 0       (不允许只转不走)
    //     |Vx| >= v_min  →  |Vz| <= |Vx| / R_min (走圆弧)
    // 注意: 不能用"Vx/Vz 同比例缩小"来限曲率 —— 分子分母同时缩小,
    //       转弯半径完全不变, 那只是整体降速, 不是曲率限制。
    double native_min_turning_radius_ = 1.0; // m: 实测最小转弯半径 (未标定前保守取 1.0)
    double max_forward_speed_ = 0.15;        // m/s: 前进速度上限
    double max_reverse_speed_ = 0.08;        // m/s: 倒车速度上限
    double max_angular_vel_ = 0.20;          // rad/s: 角速度上限
    double min_moving_speed_ = 0.005;        // m/s: 最低行驶速度 (V2.17 替代原 spin_vx_threshold_)
    double max_linear_accel_ = 0.15;         // m/s²: 整车线加速度上限 (下发前斜坡)
    double max_angular_accel_ = 0.30;        // rad/s²: 整车角加速度上限 (下发前斜坡)

    // ---- CAN & 线程 ----
    int can_socket_ = -1;

    // ============================================================
    // manda V2.11: 单实例锁 (防"两个底盘节点抢 can0")
    // ------------------------------------------------------------
    // 背景 (2026-09-16 实测事故):
    //   19:58 起过一次"只带底盘"的 launch, 20:03 未先 kill 就再起完整导航,
    //   于是两个 chassis_bridge_node 同时向 can0 发 0x402:
    //     旧节点 A: Vx=0, Vz=0  (停车帧)
    //     新节点 B: Vx=0.15, Vz=±0.3 (真实速度帧)
    //   VCU 只认最近收到的一帧, 于是在两路命令间反复跳变 →
    //   车表现为"走一下停一下"的一顿一顿 + 转向左右摇摆 + 轮胎刮滑。
    //   现象极具误导性: 单看任一节点日志都"正常", 参数怎么调都没用。
    //
    // 为什么用 flock 而不是单纯的"已存在则退出":
    //   1) 文件锁由内核在进程退出时自动释放 —— 即便 kill -9 也不会残留
    //      永久锁死 (pid 文件方案会因崩溃残留而死锁);
    //   2) LOCK_NB 非阻塞, 第二个实例立即失败, 不会挂住 launch;
    //   3) 跨进程可靠, 不依赖 ROS_DOMAIN_ID 是否一致 (不同 domain 下
    //      ros2 node list 看不到对方, 但两个进程都仍能直接写 can0)。
    // 注意: 锁文件路径含 CAN 设备名 —— 不同 can 接口可各自单实例。
    // ============================================================
    int singleton_lock_fd_ = -1;
    std::string singleton_lock_path_;   // 锁文件路径 (仅诊断打印用)
    bool acquireSingletonLock(const std::string & can_dev);
    void releaseSingletonLock();

    std::thread receive_thread_;
    std::thread velocity_thread_;
    // manda V2.06: 线程运行标志改为 atomic —— 被主线程 (析构) 与工作线程同时读写
    std::atomic_bool receive_running_{true};

    // ---- ROS 2 接口 ----
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    // manda V2.06.1: 删除未使用的 tf_broadcaster_。
    // odom_combined → base_footprint 的 TF 由 EKF 发布 (ekf.yaml publish_tf=true),
    // 底盘桥若再发布同一 TF 会造成 TF 抖动/点云跳动 (V2.04 已因此排查过重复发布者)。
    // 本节点只发布 /odom_wheel 话题, 不广播 TF。

    // ---- 速度目标 (阿克曼仅使用 linear.x 与 angular.z) ----
    // manda V2.06: 同上, 原子化避免析构与 velocityLoop 之间的数据竞争
    std::atomic_bool velocity_running_{true};
    float target_vx_ = 0.0f, target_vz_ = 0.0f;  // cmd_vel 目标 (已 clamp)
    float cmd_vx_    = 0.0f, cmd_vz_    = 0.0f;  // 经加速度斜坡后实际下发的整车速度
    mutable std::mutex velocity_mutex_;
    // manda V2.06: 心跳时间戳改为 atomic + steady_clock 毫秒数。
    // 原为 rclcpp::Time, 被 cmdVelCallback(线程A) 写、watchdogCheck(定时器) 读,
    // 存在数据竞争; 且 rclcpp::Time 非平凡类型不能 atomic, 故存 int64 毫秒。
    std::atomic<int64_t> last_cmd_time_ms_{0};
    static int64_t steadyNowMs();   // steady_clock 当前毫秒数

    // ---- 安全状态 ----
    // manda V2.06: 改为 atomic —— 这两个标志被两个不同线程读写:
    //   - canReceiveLoop (0x100/0x103) 写入
    //   - velocityLoop / watchdogCheck 读取
    // 用 atomic 消除数据竞争 (值本身语义简单, 无需互斥锁开销)。
    std::atomic_bool is_estopped_{false};       // 0x103 故障触发
    std::atomic_bool hw_estop_active_{false};   // 0x100 硬件急停

    // ============================================================
    // 运动状态机 (纯 0x01 前后轮阿克曼 —— 无任何自旋状态)
    // ------------------------------------------------------------
    // 协议 (V1.24):
    //   0x401 Byte[0]   = 0x01/02/04/08/10  切换运动模式
    //   0x100 Byte[1]   = 0x00/01/02        待机 / 遥控器控制 / 指令控制
    //   0x104 Byte[6]   = 0x01/02/04/08/10  底盘上报的当前运动模式
    //   0x104 Byte[7]   = 0x00/01           切换完成 / 切换中
    // 注意: 0x100 Byte[1] 与 0x104 Byte[6] 都会出现 0x02, 但含义完全不同
    //       (前者 = 控制权, 后者 = 斜移运动模式), 不可混用。
    //
    // 状态迁移表 (V2.19 双模式: 0x01 阿克曼 + 0x04 原地自旋):
    //   WAIT_CONTROL      0x100 Byte[1]==0x02          → PREPARE_01
    //   PREPARE_01        静默满 250ms                 → 发一次 0x401#01 → WAIT_01
    //   WAIT_01           0x104=(01,00) 且静默满       → ACTIVE_01
    //   ACTIVE_01         普通行驶                     周期发 0x402(Vx,0,Vz)
    //   ACTIVE_01         连续 N 帧纯旋转意图          → PREPARE_04
    //   PREPARE_04        零速已发且反馈静止            → 发一次 0x401#04 → WAIT_04
    //   WAIT_04           0x104=(04,00) 且静默满       → ACTIVE_04
    //   ACTIVE_04         原地旋转                     周期发 0x402(0,0,Vz)
    //   ACTIVE_04         旋转意图消失/收到前进命令      → PREPARE_RETURN_01
    //   PREPARE_RETURN_01 零速已发且反馈静止            → 发一次 0x401#01 → WAIT_RETURN_01
    //   WAIT_RETURN_01    0x104=(01,00) 且静默满       → ACTIVE_01
    //   任意运行态        遥控接管                     完全停止发送 (REMOTE_OVERRIDE)
    //
    // 【★ 2026-09-28 重大认知修正 ★】
    //   旧版本整个 0x04 通路被删除, 理由写的是"VCU 固件不支持 0x04"。
    //   该结论【已被实测推翻】: 遥控器接管期间日志出现 feedback=0x04 ×20 次、
    //   0x02 ×34 次 —— VCU 确实支持这些模式。
    //   正确的表述是:
    //     · VCU 不会因 "Vx≈0 且 Vz≠0" 【自动】进入 0x04 (2026-09-17 抓包成立)
    //     · 但显式发 0x401#04 【可以】进入 (本版实现依据)
    //     · 0x04 下 VCU 只认 0x402 的 Vz (自旋模式), 不认 Vx
    //   因此恢复 0x04 通路，可由终点航向校正和恢复 BT 的 <Spin> 触发。
    // ============================================================
    enum class MotionState {
        WAIT_CONTROL,        // 上电: 未确认控制权, 完全不发运动帧
        PREPARE_01,          // 已发停车帧: 静默 250ms 后发一次 0x401#01
        WAIT_01,             // 已发 0x401#01: 等 0x104=(01,00) 且静默期满
        ACTIVE_01,           // 0x01 已确认: 周期下发 0x402(Vx,0,Vz)
        PREPARE_04,          // 纯旋转意图已确认: 发零速, 等反馈静止后切 0x04
        WAIT_04,             // 已发 0x401#04: 等 0x104=(04,00) 且静默期满
        ACTIVE_04,           // 0x04 已确认: 周期下发 0x402(0,0,Vz) 原地自旋
        PREPARE_RETURN_01,   // 恢复: 停车, 等反馈静止后 0x401#01
        WAIT_RETURN_01,      // 已发 0x401#01: 等 0x104=(01,00) 且静默期满
        REMOTE_OVERRIDE,     // 人工接管 (0x100 Byte[1]!=0x02): 完全静默
        MODE_ERROR           // 切换超时/模式异常且恢复预算耗尽: 锁存, 需人工介入
    };
    MotionState motion_state_ = MotionState::WAIT_CONTROL;

    // ---- 模式切换计时 ----
    std::chrono::steady_clock::time_point mode_cmd_sent_ts_{}; // 最近一次 0x401 发出时刻

    // ---- 自动恢复预算 (防止 0x02/0x08/0x10 残留导致的无限循环) ----
    int resync_retry_count_ = 0;   // 已消耗的自动恢复次数
    static constexpr int kMaxAutoRecovery = 2;

    // 稳定态模式异常需"连续 N 帧"确认, 避免 0x104 切换边缘的单帧旧值
    // 就白白消耗掉恢复预算 (0x104 以 ~100Hz 上报)。
    static constexpr uint8_t kModeMismatchConfirmFrames = 3;
    uint8_t mode_mismatch_streak_ = 0;   // 连续模式不符帧计数 (恢复正常时复位)

    // ============================================================
    // 原地自旋 (0x04) 相关 —— 由"纯旋转意图"触发
    // ------------------------------------------------------------
    // 触发判据 (在 cmdVelCallback 入口层, 【必须早于 min_moving_speed_ 死区】):
    //   spin_intent  = |vx| <= spin_vx_epsilon_ 且 |wz| >= spin_wz_enter_
    //   tiny_residual= |vx| <= spin_vx_epsilon_ 且 |wz| <  spin_wz_enter_
    // tiny_residual (如 vx=0.0002 wz=0.0004) 直接归零, 绝不切模式。
    // spin_intent 需连续 spin_enter_confirm_frames_ 帧才申请切 0x04。
    // ============================================================
    double spin_vx_epsilon_ = 0.002;         // m/s: 视为"无前进"的阈值
    double spin_wz_enter_   = 0.05;          // rad/s: 进入自旋的最小角速度
                                             // ★2026-09-28: 0.005->0.05。实测 /cmd_vel_nav
                                             //   微调残余 |wz| 最大 0.0281、中位 0.0154,
                                             //   0.005 会误判残余为旋转意图 -> 切 0x04 ->
                                             //   丢 Vx -> 位置不收敛 -> 死锁 ABORTED。
                                             //   取 0.05 < 0.12(Shim 真旋转), 两者可区分。
    double spin_wz_exit_    = 0.002;         // rad/s: 退出自旋的角速度阈值
    double max_spin_angular_vel_ = 0.20;     // rad/s: 0x04 下角速度硬上限
    int spin_enter_confirm_frames_ = 3;      // 连续 N 帧纯旋转意图才切 0x04
    int spin_exit_confirm_frames_  = 2;      // 连续 N 帧无意图才切回 0x01
    int spin_enter_counter_ = 0;             // 纯旋转意图连续帧计数
    int spin_exit_counter_  = 0;             // 退出意图连续帧计数
    double pending_spin_wz_ = 0.0;           // 0x04 目标角速度 (已限幅)
    double last_spin_wz_   = 0.0;            // 最近一次有效自旋角速度 (诊断用)
    bool   zero_frame_sent_ = false;         // 本状态是否已发过零速帧
    bool   mode_confirmed_ = false;          // 本状态是否已收到 0x104 确认

    // 反馈门控: 用于判断"车是否已静止"以安全切模式
    bool feedbackStopped() const;

    // ---- cmd_vel 看门狗 (不依赖 ROS 时间, 用 steady_clock) ----
    // 安全关键: 若测试程序意外停止 (Ctrl+C / 崩溃), 必须立刻停转,
    //   否则小车会持续用最后一次 Vz 旋转。
    std::chrono::steady_clock::time_point last_cmd_time_{};
    double cmd_vel_timeout_ = 0.30;   // s: 超时后 ACTIVE_04 立即切回 0x01

    // ---- 总开关 (默认为 false, 确保新代码首次启动仍是纯 0x01 行为) ----
    bool enable_spin_mode_ = false;

    // 进入新状态 (统一入口, 复位计时器/标志)
    void enterState(MotionState s);

    // ============================================================
    // [LEGACY 0x08] 先转向后驱动 + 轮速斜坡 (仅回退参考, 运行路径不调用)
    // ------------------------------------------------------------
    // 0x08 下上位机自己控制 0x404 转角, 所以需要"先把前轮摆到位再给轮速"
    // 的协调逻辑。换成 0x01 后转向协调由 VCU 完成, 上位机只需限制整车
    // 速度与加速度 (max_linear_accel_ / max_angular_accel_), 本段不再生效。
    // ============================================================
    enum class DriveState {
        NORMAL_DRIVE,
        PRE_STEER,
        RAMP_DRIVE,
        STEER_ERROR
    };
    DriveState drive_state_ = DriveState::NORMAL_DRIVE;

    double max_wheel_accel_ = 0.15;         // m/s² 轮速斜坡上限
    double steer_ready_tolerance_ = 2.0;    // 度: 前轮转角误差容差
    int    steer_ready_frames_ = 3;         // 连续满足帧数
    double steer_timeout_ = 1.5;            // 秒: 转向等待超时
    int    steer_ok_counter_ = 0;           // 连续满足计数
    double ramp_current_speed_ = 0.0;       // 当前斜坡速度 (m/s)
    bool   ramp_active_ = false;            // 斜坡是否激活
    std::chrono::steady_clock::time_point steer_start_ts_{}; // 进入 PRE_STEER 时刻
    float target_angles_[4] = {0, 0, 0, 0}; // 目标轮角 (度)
    float target_speeds_[4] = {0, 0, 0, 0}; // 目标轮速 (m/s)

    bool isSteerReady();                                            // [LEGACY]
    void applySpeedRamp(float speeds[4], float target_speeds[4], double dt);  // [LEGACY]


    // ---- manda V2.08: 控制权 (0x100 Byte[1]) ----
    // 0x00=待机 / 0x01=遥控器控制(人工接管) / 0x02=指令控制(CAN)
    // control_mode_known_: 是否已收到过至少一帧 0x100 并确认控制权。
    //   在确认之前 (WAIT_CONTROL) 绝对不发任何运动帧 —— 否则可能在
    //   遥控器控制期间误发 401/402, 与操作者争权或造成意外运动。
    // 锁存语义: 一旦检测到人工接管 → REMOTE_OVERRIDE, 完全不发运动帧,
    //   直到控制权交还 (Byte[1] 回 0x02) 才重新走 PREPARE_01 → 0x401#01 完整时序。
    // 设计取舍: 不做"自动抢回控制权"(周期发 401#01), 因为
    //   1) 遥控器是人工安全接管, 软件不应与操作者争权;
    //   2) 周期发 401 会破坏状态机要求的 250ms 静默窗口;
    //   3) 0x401 是"运动模式"切换, 未必能夺回 Byte[1] 表示的"控制权";
    //   4) 自动恢复后若继续执行旧 cmd_vel, 车辆可能突然运动。
    std::atomic_bool    control_mode_known_{false};
    std::atomic<uint8_t> control_mode_{0};
    std::atomic_bool    remote_override_{false};
    uint8_t last_control_mode_ = 0;   // 最近一次 0x100 Byte[1] (仅 canReceiveLoop 线程读写)

    // ---- 统一发送门控 ----
    // 所有 0x401~0x404 运动帧都必须经过此判定, 防止竞态下在遥控接管期间漏发。
    // 不满足时返回 false 且不发送任何帧。
    bool allowMotionTx() const {
        return control_mode_known_.load() &&
               control_mode_.load() == 0x02 &&
               !remote_override_.load();
    }

    // ---- 数值斜坡工具 (加速度限制) ----
    static float approach(float current, float target, double max_delta) {
        const float t = target;
        const float d = static_cast<float>(max_delta);
        if (current < t) return std::min(current + d, t);
        if (current > t) return std::max(current - d, t);
        return t;
    }

    // ---- steady_clock 计时 (协议计时不随 ROS 时间跳变) ----
    std::chrono::steady_clock::time_point state_enter_ts_;   // 进入当前状态时刻

    // ---- 模式切换/恢复参数 (declare 于构造, 可被 yaml 覆盖) ----
    // 【V2.18】已删除全部与原地旋转相关的成员参数。
    // 取代它们的是单一的 min_moving_speed_ (已在上面整车运动限制中声明):
    //   |Vx| < min_moving_speed_  → Vx=0, Vz=0   (兼作停车归零阈值)
    double mode_switch_silence_ms_ = 250.0;  // ms: 0x401 前后静默期 (协议要求 200ms)
    double mode_switch_timeout_ms_ = 1500.0; // ms: 等待 0x104 模式确认超时

    // ---- 0x402 整车速度控制 (本车唯一的下发帧) ----
    bool sendVelocityCommand(float vx, float vy, float vz);

    // ---- 0x401 模式切换 ----
    // ★ 2026-09-28: 由无参改为带目标模式, 以支持 0x01 ↔ 0x04 安全切换。
    //   调用方只应传 manda_mode::FRONT_REAR_ACKERMANN 或 manda_mode::SPIN,
    //   本函数内部不做合法性判断以外的模式改写。
    //   注意: 发本帧前后必须保持 >= mode_switch_silence_ms_ 的完全静默,
    //         否则底盘会放弃模式切换 (协议硬性要求 >=200ms)。
    bool sendModeSwitch(uint8_t target_mode);

    // 兼容旧调用点: 等价于 sendModeSwitch(FRONT_REAR_ACKERMANN)
    bool sendAckermannModeSwitch() {
        return sendModeSwitch(manda_mode::FRONT_REAR_ACKERMANN);
    }

    // ---- 里程计 ----
    double x_ = 0.0, y_ = 0.0, th_ = 0.0;
    rclcpp::Time last_time_;

    // ---- 关节累计角度 (用于 joint_states 位置, 单位 rad) ----
    double wheel_pos_[4] = {0};  // rad
    double steer_pos_[4] = {0};  // rad
    rclcpp::Time last_joint_time_;

    // ---- 帧 ID ----
    // manda 修复: 与 EKF (ekf.yaml) 一致, 避免 frame 混淆
    std::string odom_frame_id_ = "odom_combined";
    std::string base_frame_id_ = "base_footprint";

    // ---- 0x104 角速度标度修正 ----
    // 底盘固件 vz 可能不是 rad/s 或标度不匹配, 用此系数修正 Yaw 积分。
    // ★ 2026-09-28: 恢复 0x04 分支。0x04 的实际标度【尚未标定】,
    //   先用 1.0 起步 —— spin04 测试 (scripts/test_spin_mode_odom.py) 可验证:
    //   若 0x04 下 /odom_wheel 与 IMU 陀螺差异大, 按比例修正本值。
    double vz_scale_mode_01_ = 1.0;   // 0x01 前后轮阿克曼 (已标定, 保持 1.0)
    double vz_scale_mode_04_ = 1.0;   // 0x04 原地自旋 (待标定)
    double vz_scale_mode_08_ = 1.84;  // 0x08 逐轮控制 (历史值, 仅 LEGACY 回退参考)
    double vzScaleForMode(uint8_t mode) const;   // 按模式选择 vz 标度系数

    // ---- 循环/看门狗周期 (参数化) ----
    int loop_ms_ = 100;            // 下发 0x403/0x404 周期
    double watchdog_timeout_ = 0.5; // cmd_vel 心跳超时 (秒)

    ChassisState state_;

    // ============================================================
    // 数据转换工具
    // ============================================================
    static inline void speedToCanBytes(float speed_mps, uint8_t &hi, uint8_t &lo);
    static inline void angleToCanBytes(float angle_deg, uint8_t &hi, uint8_t &lo);
    static inline float canBytesToSpeed(uint8_t hi, uint8_t lo);
    static inline float canBytesToAngle(uint8_t hi, uint8_t lo);

    // ============================================================
    // [LEGACY 0x08] 纯阿克曼逆运动学 (上位机逐轮解算)
    // ------------------------------------------------------------
    // 仅在 0x08 (用户逐轮控制) 模式下才需要, V2.08 起运行路径不再调用,
    // 保留定义作为回退参考。
    // ============================================================
    void computeAckermannKinematics(float vx, float omega,
                                    float speeds[4], float angles[4]);

    // ============================================================
    // CAN 基础发送
    // ============================================================
    bool sendFrame(const struct can_frame &frame);
    bool sendWheelSpeeds(float v_fl, float v_rl, float v_fr, float v_rr);
    bool sendSteerAngles(float a_fl, float a_rl, float a_fr, float a_rr);
    bool sendAckermannCommand(float speeds[4], float angles[4]);

    // ============================================================
    // 回调 & 循环
    // ============================================================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void watchdogCheck();
    void velocityLoop();
    void canReceiveLoop();
    void publishJointStates();

    // ============================================================
    // 运动状态机 (纯 0x01 前后轮阿克曼)
    // ============================================================
    void updateMotionStateMachine();     // 由 velocityLoop 每周期调用 (锁内)
    void onModeFeedback(uint8_t mode, uint8_t switch_flag);  // 由 0x104 处理调用

    // 按"底盘当前实际模式"发送匹配的零速帧 (停车/异常恢复/接管前用):
    //   mode==0x08  → 0x403 四轮线速度全 0 (LEGACY)
    //   其余         → 0x402 Vx=Vy=Vz=0
    void sendZeroForConfirmedMode(uint8_t mode);

    // 模式异常/切换失败时的统一恢复入口:
    // 先按实际模式停车 → 静默 250ms → 0x401#01; 恢复预算耗尽则进 MODE_ERROR。
    void scheduleRecoveryTo01(const char *reason, uint8_t observed_mode);
};

#endif  // __MANDA_CAN_BRIDGE_H_
