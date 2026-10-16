#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>
#include <sys/select.h>
#include <unistd.h>

// 默认航点输出目录（由 CMakeLists 注入，指向本包源码目录下的 waypoints/）
#ifndef WP_DEFAULT_WAYPOINTS_DIR
#define WP_DEFAULT_WAYPOINTS_DIR "."
#endif

// ============================================================
// position_recorder — 位姿采点记录器
//
// 订阅定位话题，把当前底盘位姿追加保存为 JSON 航点，
// 供 json_waypoint_navigator 读取执行。
//
// 保存触发方式：
//   1) 服务调用:  ros2 service call /save_position std_srvs/srv/Trigger
//   2) 终端输入:  在运行该节点的终端输入 r 并回车
//   3) 定时自动:  -p auto_save:=true -p auto_save_interval:=10.0
//
// 常用参数：
//   topic_type    amcl_pose(默认) / odom / pose_stamped
//   topic_name    /amcl_pose(默认)
//   output_file   current_position.json(默认)
//   save_directory 默认 WP_DEFAULT_WAYPOINTS_DIR
// ============================================================

using json = nlohmann::json;
using namespace std::chrono_literals;
class PositionRecorder : public rclcpp::Node
{
public:
  PositionRecorder()
  : Node("position_recorder")
  {
    // 声明参数
    output_file_ = this->declare_parameter<std::string>(
      "output_file", 
      "current_position.json"
    );
    
    topic_name_ = this->declare_parameter<std::string>(
      "topic_name", 
      "/amcl_pose"
    );
    
    topic_type_ = this->declare_parameter<std::string>(
      "topic_type", 
      "amcl_pose"  // 可选: "amcl_pose", "odom", "pose_stamped"
    );
    
    frame_id_ = this->declare_parameter<std::string>(
      "frame_id", 
      "map"
    );

    save_directory_ = this->declare_parameter<std::string>(
      "save_directory",
      WP_DEFAULT_WAYPOINTS_DIR
    );
    
    auto_save_ = this->declare_parameter<bool>(
      "auto_save", 
      false
    );
    
    auto_save_interval_ = this->declare_parameter<double>(
      "auto_save_interval", 
      5.0
    );

    // 创建订阅者
    if (topic_type_ == "amcl_pose") {
      auto amcl_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      amcl_qos.reliable();
      amcl_qos.transient_local();
      amcl_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_name_, 
        amcl_qos,
        std::bind(&PositionRecorder::amcl_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(get_logger(), "订阅AMCL定位话题: %s", topic_name_.c_str());
      RCLCPP_INFO(get_logger(), "AMCL订阅QoS: reliable + transient_local（可接收最近一次定位结果）");
    } 
    else if (topic_type_ == "odom") {
      auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(20));
      odom_qos.reliable();
      odom_qos.durability_volatile();
      odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name_, 
        odom_qos,
        std::bind(&PositionRecorder::odom_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(get_logger(), "订阅里程计话题: %s", topic_name_.c_str());
    }
    else if (topic_type_ == "pose_stamped") {
      auto pose_qos = rclcpp::QoS(rclcpp::KeepLast(10));
      pose_qos.reliable();
      pose_qos.durability_volatile();
      pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name_, 
        pose_qos,
        std::bind(&PositionRecorder::pose_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(get_logger(), "订阅PoseStamped话题: %s", topic_name_.c_str());
    }
    else {
      RCLCPP_ERROR(get_logger(), "不支持的话题类型: %s", topic_type_.c_str());
      RCLCPP_INFO(get_logger(), "支持的类型: amcl_pose, odom, pose_stamped");
      return;
    }

    // 创建保存位置的服务
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_position",
      std::bind(&PositionRecorder::save_position_callback, this,
                std::placeholders::_1, std::placeholders::_2)
    );

    // 创建定时器用于自动保存
    if (auto_save_) {
      auto_save_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(auto_save_interval_),
        std::bind(&PositionRecorder::auto_save_callback, this)
      );
      RCLCPP_INFO(get_logger(), "自动保存已启用，间隔: %.1f 秒", auto_save_interval_);
    }

    RCLCPP_INFO(get_logger(), "位置记录器已启动");
    RCLCPP_INFO(get_logger(), "输出文件: %s", output_file_.c_str());
    RCLCPP_INFO(get_logger(), "保存目录: %s", save_directory_.c_str());
    RCLCPP_INFO(get_logger(), "按Ctrl+C退出，或调用服务保存位置");
    RCLCPP_INFO(get_logger(), "等待首条位姿数据中... 当前话题发布者数量: %zu", this->count_publishers(topic_name_));

    // 启动终端输入线程：输入 r 或 R 后回车可手动保存当前位置
    input_thread_ = std::thread(&PositionRecorder::input_loop, this);

    // 周期性诊断：首条数据到达前输出更明确提示
    startup_diagnose_timer_ = this->create_wall_timer(
      5s,
      std::bind(&PositionRecorder::startup_diagnose_callback, this)
    );
  }

  ~PositionRecorder() override
  {
    keep_running_.store(false);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  struct PositionData {
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    std::string frame_id;
    std::chrono::system_clock::time_point timestamp;
    std::string source_topic;
  };

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr auto_save_timer_;
  rclcpp::TimerBase::SharedPtr startup_diagnose_timer_;
  
  std::string output_file_;
  std::string topic_name_;
  std::string topic_type_;
  std::string frame_id_;
  std::string save_directory_;
  bool auto_save_;
  double auto_save_interval_;
  
  PositionData current_position_;
  bool has_position_data_ = false;
  bool first_data_logged_ = false;
  std::mutex position_mutex_;
  std::thread input_thread_;
  std::atomic<bool> keep_running_{true};

  std::string build_full_output_path() const
  {
    if (!save_directory_.empty() && save_directory_.back() == '/') {
      return save_directory_ + output_file_;
    }
    return save_directory_ + "/" + output_file_;
  }

  void input_loop()
  {
    RCLCPP_INFO(get_logger(), "终端输入已启用：输入 r 并回车可保存当前位置");

    while (rclcpp::ok() && keep_running_.load()) {
      fd_set readfds;
      FD_ZERO(&readfds);
      FD_SET(STDIN_FILENO, &readfds);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 200000;

      const int ret = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
      if (ret <= 0) {
        continue;
      }

      if (FD_ISSET(STDIN_FILENO, &readfds)) {
        char c = 0;
        const ssize_t bytes_read = read(STDIN_FILENO, &c, 1);
        if (bytes_read <= 0) {
          continue;
        }

        if (c == 'r' || c == 'R') {
          if (save_position_to_file()) {
            RCLCPP_INFO(get_logger(), "终端触发保存成功");
          } else {
            RCLCPP_WARN(get_logger(), "终端触发保存失败（可能还没有位置数据）");
          }
        }
      }
    }
  }

  // 四元数转欧拉角
  void quaternionToEuler(double x, double y, double z, double w, 
                         double& roll, double& pitch, double& yaw)
  {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
      pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
      pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void update_position(const geometry_msgs::msg::Pose& pose, 
                       const std::string& frame_id,
                       const std::string& source)
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    
    current_position_.x = pose.position.x;
    current_position_.y = pose.position.y;
    current_position_.z = pose.position.z;
    current_position_.frame_id = frame_id;
    current_position_.timestamp = std::chrono::system_clock::now();
    current_position_.source_topic = source;
    
    // 转换四元数为欧拉角
    quaternionToEuler(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w,
      current_position_.roll,
      current_position_.pitch,
      current_position_.yaw
    );
    
    has_position_data_ = true;

    if (!first_data_logged_) {
      first_data_logged_ = true;
      RCLCPP_INFO(get_logger(),
                  "已收到首条位姿数据，允许保存。frame=%s, source=%s",
                  current_position_.frame_id.c_str(),
                  current_position_.source_topic.c_str());
    }
    
    // 打印当前位置信息
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000,
      "当前位置: (%.2f, %.2f, %.2f), 朝向: %.2f rad",
      current_position_.x, current_position_.y, current_position_.z,
      current_position_.yaw);
  }

  void startup_diagnose_callback()
  {
    if (has_position_data_) {
      startup_diagnose_timer_.reset();
      return;
    }

    RCLCPP_WARN(get_logger(),
                "尚未收到位姿数据，暂时无法保存。topic=%s, type=%s, 发布者数量=%zu",
                topic_name_.c_str(),
                topic_type_.c_str(),
                this->count_publishers(topic_name_));
    RCLCPP_WARN(get_logger(),
                "排查建议: 1) 确认定位节点在线 2) 确认topic_name/topic_type参数匹配 3) 尝试 'ros2 topic echo %s --once'",
                topic_name_.c_str());
  }

  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    update_position(msg->pose.pose, msg->header.frame_id, topic_name_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    update_position(msg->pose.pose, msg->header.frame_id, topic_name_);
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    update_position(msg->pose, msg->header.frame_id, topic_name_);
  }

  void save_position_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request; // 未使用参数
    
    if (!has_position_data_) {
      response->success = false;
      response->message = "没有可用的位置数据，请先确认定位话题有消息";
      RCLCPP_WARN(get_logger(),
                  "尝试保存位置但无可用数据。topic=%s, type=%s, 发布者数量=%zu",
                  topic_name_.c_str(),
                  topic_type_.c_str(),
                  this->count_publishers(topic_name_));
      return;
    }

    if (save_position_to_file()) {
      response->success = true;
      response->message = "位置已成功保存到文件";
      RCLCPP_INFO(get_logger(), "位置已保存到文件");
    } else {
      response->success = false;
      response->message = "保存位置到文件失败";
      RCLCPP_ERROR(get_logger(), "保存位置到文件失败");
    }
  }

  void auto_save_callback()
  {
    if (has_position_data_) {
      if (save_position_to_file()) {
        RCLCPP_INFO(get_logger(), "自动保存位置到文件");
      } else {
        RCLCPP_ERROR(get_logger(), "自动保存位置失败");
      }
    }
  }

  bool save_position_to_file()
  {
    std::lock_guard<std::mutex> lock(position_mutex_);
    
    if (!has_position_data_) {
      return false;
    }

    // 构建完整文件路径
    const std::string full_path = build_full_output_path();
    
    RCLCPP_INFO(get_logger(), 
                "保存位置到文件: %s", 
                full_path.c_str());

    try {
      // 当前位姿点（字段与导航节点读取的格式保持一致）
      const json new_position = {
        {"x", current_position_.x},
        {"y", current_position_.y},
        {"z", current_position_.z},
        {"roll", current_position_.roll},
        {"pitch", current_position_.pitch},
        {"yaw", current_position_.yaw},
        {"enable_refine", false}
      };

      // 读取历史数据并追加，支持旧格式：position为单个对象时自动转为数组
      json position_json;
      position_json["position"] = json::array();

      {
        std::ifstream input_file(full_path);
        if (input_file.is_open() && input_file.peek() != std::ifstream::traits_type::eof()) {
          json existing_json;
          input_file >> existing_json;
          if (existing_json.contains("position")) {
            if (existing_json["position"].is_array()) {
              position_json["position"] = existing_json["position"];
            } else if (existing_json["position"].is_object()) {
              position_json["position"].push_back(existing_json["position"]);
            }
          }
        }
      }

      position_json["position"].push_back(new_position);
      
      // 写入文件
      std::ofstream file(full_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(get_logger(), 
                     "无法打开文件: %s", 
                     full_path.c_str());
        return false;
      }
      
      file << position_json.dump(2); // 使用2个空格缩进
      file.close();
      
      RCLCPP_INFO(get_logger(), 
                  "路径点已保存(总计%zu个): (%.3f, %.3f, %.3f), 朝向: %.3f rad",
                  position_json["position"].size(),
                  current_position_.x, current_position_.y, current_position_.z,
                  current_position_.yaw);
      return true;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), 
                   "保存位置时出错: %s", 
                   e.what());
      return false;
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<PositionRecorder>();
  
  // 打印使用说明
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "位置记录器使用说明:");
  RCLCPP_INFO(node->get_logger(), "1. 调用服务保存当前位置:");
  RCLCPP_INFO(node->get_logger(), "   ros2 service call /save_position std_srvs/srv/Trigger");
  RCLCPP_INFO(node->get_logger(), "2. 或使用参数自动保存:");
  RCLCPP_INFO(node->get_logger(), "   --ros-args -p auto_save:=true -p auto_save_interval:=10.0");
  RCLCPP_INFO(node->get_logger(), "3. 更改话题类型:");
  RCLCPP_INFO(node->get_logger(), "   --ros-args -p topic_type:=odom -p topic_name:=/odom");
  RCLCPP_INFO(node->get_logger(), "4. 在当前终端输入 r 并回车可立即保存当前位置");
  RCLCPP_INFO(node->get_logger(), "5. 更改输出目录:");
  RCLCPP_INFO(node->get_logger(), "   --ros-args -p save_directory:=/abs/dir");
  RCLCPP_INFO(node->get_logger(), "========================================");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}                                                                         