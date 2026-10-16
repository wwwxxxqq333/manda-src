cmake_minimum_required(VERSION 3.8)
project(waypoint_action_publisher)

# 查找所有需要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 如果你已安装系统级的 nlohmann-json3-dev，可以这样找到它：
find_package(nlohmann_json REQUIRED)

# 包含自定义头文件目录
include_directories(include)

# 两个节点共用的依赖集合
set(COMMON_DEPS
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 第一个可执行文件：json_nav_action_publisher.cpp
add_executable(json_waypoint_navigator src/json_nav_action_publisher.cpp)
target_link_libraries(json_waypoint_navigator nlohmann_json::nlohmann_json)
ament_target_dependencies(json_waypoint_navigator ${COMMON_DEPS})

# 第二个可执行文件：position_recorder.cpp
add_executable(position_recorder src/position_recorder.cpp)
target_link_libraries(position_recorder nlohmann_json::nlohmann_json)
ament_target_dependencies(position_recorder ${COMMON_DEPS})

# 默认航点目录：包源码目录下的 waypoints/
# 放在源码树而非 install/，避免 colcon build 清空数据；
# 运行时可用参数 waypoints_file / save_directory 覆盖。
set(WAYPOINTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/waypoints")
file(MAKE_DIRECTORY "${WAYPOINTS_DIR}")
target_compile_definitions(json_waypoint_navigator PRIVATE
  WP_DEFAULT_WAYPOINTS_DIR="${WAYPOINTS_DIR}")
target_compile_definitions(position_recorder PRIVATE
  WP_DEFAULT_WAYPOINTS_DIR="${WAYPOINTS_DIR}")

# 安装可执行文件
install(TARGETS
  json_waypoint_navigator
  position_recorder
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装一份航点目录副本（仅作模板参考，真实数据在源码树 waypoints/）
install(DIRECTORY waypoints
  DESTINATION share/${PROJECT_NAME}
)

# 如有需要，安装自定义头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 测试设置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 最后打包
ament_package()