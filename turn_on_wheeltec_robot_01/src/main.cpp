#include "turn_on_wheeltec_robot_01/manda_can_bridge.h"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // manda V2.11: 构造函数可能因"另一个底盘节点已独占 can0"抛出异常。
    // 必须在这里接住并返回非零码, 让 launch 明确报错终止 ——
    // 否则会出现"看似启动成功、实际两个驱动源抢总线"的隐蔽故障
    // (表现为车辆一顿一顿、左右摇摆, 但每个节点自己的日志都正常)。
    std::shared_ptr<MandaCanBridge> node;
    try {
        node = std::make_shared<MandaCanBridge>();
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("chassis_bridge_node"),
            "启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1;   // 非零退出码 → launch 记为失败
    }

    rclcpp::spin(node);

    // 显式重置, 保证析构函数 (释放 flock 单实例锁) 在 shutdown 前完成,
    // 让下一次启动能立刻拿到锁。
    node.reset();

    rclcpp::shutdown();
    return 0;
}
