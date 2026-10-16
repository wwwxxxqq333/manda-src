// ============================================================
// json_nav_action_publisher.cpp — JSON 航点导航器
//
// 读取 JSON 航点文件，逐个通过 Nav2 的 NavigateToPose 动作导航到点，
// 并用 TF 校验实际到点精度（距离 + 朝向角误差），可选到点精修。
//
// 用法：
//   ros2 run waypoint_action_publisher json_waypoint_navigator \
//     --ros-args -p waypoints_file:=/abs/path/current_position.json
//
// JSON 支持两种格式（二选一）：
//   1) {"waypoints": [{"name":"a", "x":1.0, "y":2.0, "yaw":0.0,
//                      "wait_time":3.0, "enable_refine":true,
//                      "rotate_in_place":false, "approach_yaw":0.0}, ...]}
//   2) {"position":  [{"x":1.0, "y":2.0, "z":0.0, "yaw":0.0, ...}, ...]}
//      （position 也可以是单个对象，兼容旧格式）
//
// 窄通道掉头航点设置 rotate_in_place=true 和 approach_yaw：
// 先以 approach_yaw 到达目标位置，再以底盘 0x04 原地旋转到 yaw。
//
// 默认 waypoints_file 指向包源码目录下的 waypoints/current_position.json
// （由 CMakeLists 注入 WP_DEFAULT_WAYPOINTS_DIR，可在构建时覆盖）。
// 实际使用建议用参数指向你自己的航点文件。
// ============================================================

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

class JsonWaypointNavigator : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit JsonWaypointNavigator(const std::string & node_name = "json_waypoint_navigator")
  : Node(node_name)
  {
    // ---- 航点文件与坐标系 ----
    waypoints_file_ = this->declare_parameter<std::string>("waypoints_file", "");
    if (waypoints_file_.empty()) {
      waypoints_file_ = defaultWaypointsFile();
    }

    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

    // 注意：本工程基座坐标系是 base_footprint（URDF / AMCL / Nav2 全部使用），
    // 不是 base_link。若更换机器人请用参数覆盖。
    robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame", "base_footprint");

    // ---- 流程控制 ----
    wait_between_points_ = this->declare_parameter<bool>("wait_between_points", true);

    // ---- 到点判定阈值 ----
    // 必须与 Nav2 的 goal_checker 对齐，否则会一直判定"未到点"并触发无谓精修：
    // nav2_manda_2d.yaml -> xy_goal_tolerance=0.30, yaw_goal_tolerance=0.60
    position_tolerance_m_ = this->declare_parameter<double>("position_tolerance_m", 0.30);
    yaw_tolerance_rad_ = this->declare_parameter<double>("yaw_tolerance_rad", 0.60);

    // ---- 到点精修 ----
    enable_pose_refine_ = this->declare_parameter<bool>("enable_pose_refine", true);
    refine_max_attempts_ = this->declare_parameter<int>("refine_max_attempts", 2);
    refine_timeout_sec_ = this->declare_parameter<int>("refine_timeout_sec", 30);
    refine_settle_time_sec_ = this->declare_parameter<double>("refine_settle_time_sec", 0.5);

    // ---- 窄通道分段旋转 ----
    rotate_topic_ = this->declare_parameter<std::string>("rotate_topic", "/cmd_vel");
    spin_speed_rad_s_ = this->declare_parameter<double>("spin_speed_rad_s", 0.08);
    spin_timeout_sec_ = this->declare_parameter<double>("spin_timeout_sec", 60.0);
    spin_yaw_tolerance_rad_ = this->declare_parameter<double>(
      "spin_yaw_tolerance_rad", 0.035);
    max_spin_position_drift_m_ = this->declare_parameter<double>(
      "max_spin_position_drift_m", 0.08);

    // ---- 超时 ----
    nav_server_wait_sec_ = this->declare_parameter<int>("nav_server_wait_sec", 30);
    nav_timeout_sec_ = this->declare_parameter<int>("nav_timeout_sec", 180);
    goal_response_timeout_sec_ = this->declare_parameter<int>("goal_response_timeout_sec", 10);

    // ---- 导航动作客户端 ----
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    rotate_pub_ = create_publisher<geometry_msgs::msg::Twist>(rotate_topic_, 10);

    // ---- TF ----
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    if (!loadWaypoints()) {
      RCLCPP_ERROR(get_logger(), "加载航点失败，节点退出");
      return;
    }

    load_ok_ = true;

    RCLCPP_INFO(get_logger(), "JSON 航点导航器已启动");
    RCLCPP_INFO(get_logger(), "航点文件: %s", waypoints_file_.c_str());
    RCLCPP_INFO(get_logger(), "已加载 %zu 个航点", waypoints_.size());
    RCLCPP_INFO(get_logger(), "全局坐标系: %s, 基座坐标系: %s",
      frame_id_.c_str(), robot_base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "航点间等待: %s", wait_between_points_ ? "启用" : "禁用");
    RCLCPP_INFO(get_logger(), "到点判定阈值: 距离<=%.3f m, 角度<=%.3f rad",
      position_tolerance_m_, yaw_tolerance_rad_);
    RCLCPP_INFO(get_logger(),
      "到点精修: %s (最多 %d 次, 单次超时 %d 秒, 静稳等待 %.2f 秒) —— 由每个航点的 enable_refine 决定是否执行",
      enable_pose_refine_ ? "可用" : "全局禁用",
      refine_max_attempts_, refine_timeout_sec_, refine_settle_time_sec_);
  }

  bool isReady() const {return load_ok_;}

  /// 依次执行全部航点，全部成功返回 true
  bool runNavigation()
  {
    if (!waitForNavServer()) {
      RCLCPP_ERROR(get_logger(), "导航服务器不可用");
      return false;
    }

    RCLCPP_INFO(get_logger(), "开始执行航点导航...");

    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & wp = waypoints_[i];
      const double target_yaw = quaternionToYaw(wp.pose.pose.orientation);
      const auto waypoint_start_time = now();
      const double progress =
        (static_cast<double>(i + 1) / static_cast<double>(waypoints_.size())) * 100.0;
      auto navigation_pose = wp.pose;
      if (wp.rotate_in_place) {
        navigation_pose.pose.orientation = eulerToQuaternion(0.0, 0.0, wp.approach_yaw);
        RCLCPP_INFO(get_logger(),
          "航点 %s 启用分段旋转: 先按进入 yaw=%.3f 到达，再原地旋转到目标 yaw=%.3f",
          wp.name.c_str(), wp.approach_yaw, target_yaw);
      }

      RCLCPP_INFO(get_logger(),
        "[%.0f%%] 开始导航到航点 %zu/%zu: %s | 位置(x=%.3f, y=%.3f, z=%.3f), "
        "导航 yaw=%.3f, 最终 yaw=%.3f | enable_refine=%s",
        progress, i + 1, waypoints_.size(), wp.name.c_str(),
        wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z,
        quaternionToYaw(navigation_pose.pose.orientation),
        target_yaw, wp.enable_refine ? "true" : "false");

      if (!navigateToPose(
          navigation_pose, wp.name, i + 1, waypoints_.size(),
          wp.rotate_in_place ? false : wp.enable_refine)) {
        RCLCPP_ERROR(get_logger(), "航点 %zu/%zu (%s) 导航失败，终止后续航点",
          i + 1, waypoints_.size(), wp.name.c_str());
        return false;
      }

      if (wp.rotate_in_place && !rotateToYaw(target_yaw)) {
        RCLCPP_ERROR(get_logger(), "航点 %s 原地旋转失败，终止后续航点", wp.name.c_str());
        return false;
      }

      if (wp.rotate_in_place) {
        std::this_thread::sleep_for(300ms);
        if (!isPositionReached(wp.pose, position_tolerance_m_, spin_yaw_tolerance_rad_)) {
          RCLCPP_ERROR(get_logger(), "航点 %s 旋转后最终位姿未通过验收", wp.name.c_str());
          return false;
        }
      }

      const double elapsed_sec = (now() - waypoint_start_time).seconds();
      RCLCPP_INFO(get_logger(), "航点 %zu/%zu (%s) 到达成功，耗时 %.2f 秒",
        i + 1, waypoints_.size(), wp.name.c_str(), elapsed_sec);

      if (i + 1 < waypoints_.size() && wait_between_points_ && wp.wait_time > 0.0) {
        RCLCPP_INFO(get_logger(), "等待 %.1f 秒后前往下一个航点", wp.wait_time);
        std::this_thread::sleep_for(std::chrono::duration<double>(wp.wait_time));
      }
    }

    RCLCPP_INFO(get_logger(), "所有航点导航完成！");
    return true;
  }

private:
  struct Waypoint
  {
    std::string name;
    geometry_msgs::msg::PoseStamped pose;
    double wait_time = 3.0;
    bool enable_refine = false;   // 该点是否允许到点精修
    bool rotate_in_place = false;
    double approach_yaw = 0.0;
  };

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotate_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<Waypoint> waypoints_;
  std::string waypoints_file_;
  std::string frame_id_;
  std::string robot_base_frame_;
  bool wait_between_points_ = true;
  double position_tolerance_m_ = 0.30;
  double yaw_tolerance_rad_ = 0.60;
  bool enable_pose_refine_ = true;
  int refine_max_attempts_ = 2;
  int refine_timeout_sec_ = 30;
  double refine_settle_time_sec_ = 0.5;
  int nav_server_wait_sec_ = 30;
  int nav_timeout_sec_ = 180;
  int goal_response_timeout_sec_ = 10;
  std::string rotate_topic_;
  double spin_speed_rad_s_ = 0.08;
  double spin_timeout_sec_ = 60.0;
  double spin_yaw_tolerance_rad_ = 0.035;
  double max_spin_position_drift_m_ = 0.08;
  bool load_ok_ = false;

  // ------------------------------------------------------------
  // 工具函数
  // ------------------------------------------------------------

  /// 未指定 waypoints_file 时的默认路径（由 CMake 注入）
  static std::string defaultWaypointsFile()
  {
#ifdef WP_DEFAULT_WAYPOINTS_DIR
    return std::string(WP_DEFAULT_WAYPOINTS_DIR) + "/current_position.json";
#else
    return "current_position.json";
#endif
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist stop;
    rotate_pub_->publish(stop);
  }

  bool rotateToYaw(double target_yaw)
  {
    publishStop();
    std::this_thread::sleep_for(200ms);

    double yaw_error = 0.0;
    double signed_yaw_error = 0.0;
    double distance = 0.0;
    geometry_msgs::msg::PoseStamped current_target;
    if (!getPoseError(current_target, distance, yaw_error, signed_yaw_error)) {
      RCLCPP_ERROR(get_logger(), "原地旋转前无法读取机器人 TF");
      return false;
    }

    const double start_x = last_cur_x_;
    const double start_y = last_cur_y_;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(spin_timeout_sec_);
    RCLCPP_INFO(get_logger(), "开始原地旋转: 当前 yaw=%.3f, 目标 yaw=%.3f", last_cur_yaw_, target_yaw);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(get_node_base_interface());
      geometry_msgs::msg::Twist command;
      command.linear.x = 0.0;
      command.linear.y = 0.0;

      geometry_msgs::msg::PoseStamped yaw_target;
      yaw_target.header.frame_id = frame_id_;
      yaw_target.pose.position.x = last_cur_x_;
      yaw_target.pose.position.y = last_cur_y_;
      yaw_target.pose.orientation = eulerToQuaternion(0.0, 0.0, target_yaw);
      if (!getPoseError(yaw_target, distance, yaw_error, signed_yaw_error)) {
        publishStop();
        return false;
      }

      const double drift = std::hypot(last_cur_x_ - start_x, last_cur_y_ - start_y);
      if (drift > max_spin_position_drift_m_) {
        publishStop();
        RCLCPP_ERROR(
          get_logger(), "原地旋转位置漂移 %.3f m 超过上限 %.3f m",
          drift, max_spin_position_drift_m_);
        return false;
      }
      if (yaw_error <= spin_yaw_tolerance_rad_) {
        publishStop();
        RCLCPP_INFO(get_logger(), "原地旋转完成: yaw 误差 %.3f rad, 漂移 %.3f m", yaw_error, drift);
        return true;
      }

      command.angular.z = signed_yaw_error >= 0.0 ? spin_speed_rad_s_ : -spin_speed_rad_s_;
      rotate_pub_->publish(command);
      std::this_thread::sleep_for(50ms);
    }

    publishStop();
    RCLCPP_ERROR(get_logger(), "原地旋转超时 (%.1f s)", spin_timeout_sec_);
    return false;
  }

  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  geometry_msgs::msg::Quaternion eulerToQuaternion(double roll, double pitch, double yaw) const
  {
    geometry_msgs::msg::Quaternion q;
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
  }

  static const char * resultCodeToString(rclcpp_action::ResultCode code)
  {
    switch (code) {
      case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
      case rclcpp_action::ResultCode::ABORTED:   return "ABORTED";
      case rclcpp_action::ResultCode::CANCELED:  return "CANCELED";
      default:                                   return "UNKNOWN";
    }
  }

  // ------------------------------------------------------------
  // 位姿校验
  // ------------------------------------------------------------

  bool getPoseError(const geometry_msgs::msg::PoseStamped & target,
    double & dist, double & yaw_error, double & signed_yaw_error)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame_id_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(1.0));

      const double cur_x = transform.transform.translation.x;
      const double cur_y = transform.transform.translation.y;

      const double dx = target.pose.position.x - cur_x;
      const double dy = target.pose.position.y - cur_y;
      dist = std::sqrt(dx * dx + dy * dy);

      const double target_yaw = quaternionToYaw(target.pose.orientation);
      const double current_yaw = tf2::getYaw(transform.transform.rotation);
      signed_yaw_error = normalizeAngle(target_yaw - current_yaw);
      yaw_error = std::abs(signed_yaw_error);

      last_cur_x_ = cur_x;
      last_cur_y_ = cur_y;
      last_cur_yaw_ = current_yaw;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "TF 查询 %s -> %s 失败: %s",
        frame_id_.c_str(), robot_base_frame_.c_str(), e.what());
      return false;
    }
  }

  bool isPositionReached(const geometry_msgs::msg::PoseStamped & target,
    double position_threshold, double yaw_threshold)
  {
    double dist = 0.0;
    double yaw_error = 0.0;
    double signed_yaw_error = 0.0;
    if (!getPoseError(target, dist, yaw_error, signed_yaw_error)) {
      return false;
    }

    RCLCPP_INFO(get_logger(),
      "位置姿态验证: 目标(%.3f, %.3f, yaw=%.3f) vs 当前(%.3f, %.3f, yaw=%.3f), 距离=%.3f 米, 角误差=%.3f 弧度",
      target.pose.position.x, target.pose.position.y,
      quaternionToYaw(target.pose.orientation),
      last_cur_x_, last_cur_y_, last_cur_yaw_,
      dist, yaw_error);

    return dist <= position_threshold && yaw_error <= yaw_threshold;
  }

  // ------------------------------------------------------------
  // 航点加载
  // ------------------------------------------------------------

  bool loadWaypoints()
  {
    waypoints_.clear();

    RCLCPP_INFO(get_logger(), "尝试加载航点文件: %s", waypoints_file_.c_str());

    std::ifstream file(waypoints_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "无法打开航点文件: %s", waypoints_file_.c_str());
      RCLCPP_ERROR(get_logger(),
        "提示: 可用参数覆盖路径，例如 -p waypoints_file:=/abs/path/current_position.json");
      return false;
    }

    try {
      const json data = json::parse(file);

      const auto fill_from_obj = [this](const json & obj, const std::string & name) {
          Waypoint wp;
          wp.name = name;
          wp.pose.header.frame_id = frame_id_;
          wp.pose.pose.position.x = obj["x"].get<double>();
          wp.pose.pose.position.y = obj["y"].get<double>();
          wp.pose.pose.position.z = obj.value("z", 0.0);
          wp.pose.pose.orientation = eulerToQuaternion(
            obj.value("roll", 0.0), obj.value("pitch", 0.0), obj.value("yaw", 0.0));
          wp.wait_time = obj.value("wait_time", 3.0);
          wp.enable_refine = obj.value("enable_refine", false);
          wp.rotate_in_place = obj.value("rotate_in_place", false);
          if (wp.rotate_in_place) {
            if (!obj.contains("approach_yaw") || !obj["approach_yaw"].is_number()) {
              throw json::type_error::create(
                302, "rotate_in_place=true requires numeric approach_yaw", &obj);
            }
            wp.approach_yaw = obj["approach_yaw"].get<double>();
          }
          return wp;
        };

      // 格式 1：{"waypoints": [...]}
      if (data.contains("waypoints") && data["waypoints"].is_array()) {
        size_t index = 0;
        for (const auto & wp_json : data["waypoints"]) {
          if (!wp_json.is_object()) {
            RCLCPP_ERROR(get_logger(), "waypoints[%zu] 不是对象", index);
            return false;
          }
          waypoints_.push_back(fill_from_obj(
            wp_json,
            wp_json.value("name", "waypoint_" + std::to_string(index + 1))));
          ++index;
        }
      }
      // 格式 2：{"position": {...}} 或 {"position": [...]}
      else if (data.contains("position")) {
        if (data["position"].is_object()) {
          waypoints_.push_back(fill_from_obj(data["position"], "position_1"));
        } else if (data["position"].is_array()) {
          size_t index = 0;
          for (const auto & pos : data["position"]) {
            if (!pos.is_object()) {
              RCLCPP_ERROR(get_logger(), "position[%zu] 不是对象", index);
              return false;
            }
            waypoints_.push_back(
              fill_from_obj(pos, "position_" + std::to_string(index + 1)));
            ++index;
          }
        } else {
          RCLCPP_ERROR(get_logger(), "JSON 格式错误: 'position' 必须是对象或数组");
          return false;
        }
      } else {
        RCLCPP_ERROR(get_logger(),
          "JSON 格式错误: 需要 'waypoints' 数组或 'position' 对象/数组");
        return false;
      }

      if (waypoints_.empty()) {
        RCLCPP_ERROR(get_logger(), "未解析到任何可用航点");
        return false;
      }

      RCLCPP_INFO(get_logger(), "成功加载 %zu 个航点", waypoints_.size());
      return true;
    } catch (const json::exception & e) {
      RCLCPP_ERROR(get_logger(), "JSON 解析错误: %s", e.what());
      return false;
    }
  }

  // ------------------------------------------------------------
  // 导航
  // ------------------------------------------------------------

  bool waitForNavServer()
  {
    RCLCPP_INFO(get_logger(), "等待导航服务器...");
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(nav_server_wait_sec_))) {
      RCLCPP_ERROR(get_logger(), "导航服务器在 %d 秒内不可用", nav_server_wait_sec_);
      return false;
    }
    RCLCPP_INFO(get_logger(), "导航服务器已就绪");
    return true;
  }

  /// 发送一次 NavigateToPose 目标并等待结果
  rclcpp_action::ResultCode sendNavGoalAndWait(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & waypoint_name,
    size_t waypoint_index,
    size_t waypoint_total,
    int timeout_sec)
  {
    auto goal_pose = pose;
    goal_pose.header.stamp = now();

    NavigateToPose::Goal goal;
    goal.pose = goal_pose;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(get_logger(), "目标被服务器拒绝");
        } else {
          RCLCPP_INFO(get_logger(), "目标已被接受");
        }
      };

    options.feedback_callback =
      [this, waypoint_name, waypoint_index, waypoint_total](
        GoalHandleNav::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        const double eta_sec =
          rclcpp::Duration(feedback->estimated_time_remaining).seconds();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "航点 %zu/%zu (%s) 进行中: 剩余距离=%.2f m, 预计剩余=%.1f s, 恢复次数=%d",
          waypoint_index, waypoint_total, waypoint_name.c_str(),
          feedback->distance_remaining, eta_sec, feedback->number_of_recoveries);
      };

    RCLCPP_INFO(get_logger(),
      "发送导航目标: 航点 %zu/%zu (%s), frame=%s, x=%.3f, y=%.3f, z=%.3f",
      waypoint_index, waypoint_total, waypoint_name.c_str(),
      goal_pose.header.frame_id.c_str(),
      goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);

    auto send_goal_future = nav_client_->async_send_goal(goal, options);
    if (rclcpp::spin_until_future_complete(shared_from_this(), send_goal_future,
      std::chrono::seconds(goal_response_timeout_sec_)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "发送目标失败（%d 秒内未收到响应）", goal_response_timeout_sec_);
      return rclcpp_action::ResultCode::ABORTED;
    }

    auto goal_handle = send_goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "目标被拒绝");
      return rclcpp_action::ResultCode::ABORTED;
    }

    auto result_future = nav_client_->async_get_result(goal_handle);
    const auto status = rclcpp::spin_until_future_complete(
      shared_from_this(), result_future, std::chrono::seconds(timeout_sec));

    if (status == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(get_logger(), "导航超时（%d 秒），取消目标", timeout_sec);
      nav_client_->async_cancel_goal(goal_handle);
      std::this_thread::sleep_for(500ms);
      return rclcpp_action::ResultCode::CANCELED;
    }
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "获取导航结果失败");
      return rclcpp_action::ResultCode::ABORTED;
    }

    return result_future.get().code;
  }

  bool navigateToPose(const geometry_msgs::msg::PoseStamped & pose,
    const std::string & waypoint_name,
    size_t waypoint_index,
    size_t waypoint_total,
    bool enable_refine)
  {
    const auto nav_start_time = now();
    const auto code = sendNavGoalAndWait(pose, waypoint_name,
      waypoint_index, waypoint_total, nav_timeout_sec_);

    if (code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(get_logger(), "Nav2 报告失败: %s", resultCodeToString(code));
      return false;
    }

    RCLCPP_INFO(get_logger(), "Nav2 报告导航成功，耗时 %.2f 秒", (now() - nav_start_time).seconds());

    if (refine_settle_time_sec_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(refine_settle_time_sec_));
    }

    // ---- 位姿校验 ----
    if (isPositionReached(pose, position_tolerance_m_, yaw_tolerance_rad_)) {
      return true;
    }

    if (!enable_refine) {
      RCLCPP_INFO(get_logger(),
        "航点 %zu/%zu (%s): enable_refine=false，跳过到点精修（阈值仅记录不强制）",
        waypoint_index, waypoint_total, waypoint_name.c_str());
      return true;
    }

    if (!enable_pose_refine_ || refine_max_attempts_ <= 0) {
      RCLCPP_ERROR(get_logger(),
        "Nav2 返回成功但位姿未满足阈值(距离<=%.3f m, 角度<=%.3f rad)，且精修已禁用",
        position_tolerance_m_, yaw_tolerance_rad_);
      return false;
    }

    RCLCPP_WARN(get_logger(),
      "航点 %zu/%zu (%s): enable_refine=true，位姿未满足阈值，开始到点精修(最多 %d 次)",
      waypoint_index, waypoint_total, waypoint_name.c_str(), refine_max_attempts_);

    for (int attempt = 1; attempt <= refine_max_attempts_; ++attempt) {
      auto refine_pose = pose;
      refine_pose.header.stamp = now();

      RCLCPP_INFO(get_logger(), "到点精修 %d/%d: 重新发送同一目标",
        attempt, refine_max_attempts_);

      const auto refine_code = sendNavGoalAndWait(refine_pose,
        waypoint_name + "(精修)", waypoint_index, waypoint_total, refine_timeout_sec_);

      if (refine_code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_WARN(get_logger(), "到点精修 %d/%d: Nav2 结果=%s",
          attempt, refine_max_attempts_, resultCodeToString(refine_code));
        continue;
      }

      if (refine_settle_time_sec_ > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(refine_settle_time_sec_));
      }

      if (isPositionReached(refine_pose, position_tolerance_m_, yaw_tolerance_rad_)) {
        RCLCPP_INFO(get_logger(), "到点精修成功，最终位姿满足阈值");
        return true;
      }

      RCLCPP_WARN(get_logger(), "到点精修 %d/%d 后仍未满足阈值，继续下一次",
        attempt, refine_max_attempts_);
    }

    RCLCPP_ERROR(get_logger(),
      "Nav2 返回成功，但经过 %d 次到点精修后仍未满足阈值(距离<=%.3f m, 角度<=%.3f rad)",
      refine_max_attempts_, position_tolerance_m_, yaw_tolerance_rad_);
    return false;
  }

  // 最近一次 TF 查询到的当前位姿（仅用于日志）
  double last_cur_x_ = 0.0;
  double last_cur_y_ = 0.0;
  double last_cur_yaw_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JsonWaypointNavigator>();

  if (!node->isReady()) {
    rclcpp::shutdown();
    return 1;
  }

  std::this_thread::sleep_for(2s);   // 等待 DDS 发现完成
  const bool ok = node->runNavigation();

  node.reset();                      // 显式析构，释放 Action Client
  std::this_thread::sleep_for(500ms);

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
