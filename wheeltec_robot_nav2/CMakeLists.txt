cmake_minimum_required(VERSION 3.5)
project(wheeltec_nav2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)

endif()
add_compile_options(-g)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
################################################################################
# manda V2.06 修复: 补上 behavior_trees
#   原先此处漏装 behavior_trees, 导致 param_manda.yaml 里
#   default_nav_to_pose_bt_xml 指向的行为树 XML 从未进入 install/,
#   Nav2 实际回退到内置默认 BT (含 RateController hz=1.0),
#   表现为"改了行为树却没生效 / 到点后反复绕圈重新规划"。
install(
  DIRECTORY launch map param rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
