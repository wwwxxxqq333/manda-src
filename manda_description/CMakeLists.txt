cmake_minimum_required(VERSION 3.8)
project(manda_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ---- Generate URDF from xacro at build time --------------------------------
find_program(XACRO_CMD xacro REQUIRED)
set(XACRO_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/urdf/fwids_vehicle.xacro")
set(URDF_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/urdf/fwids_vehicle.urdf")

add_custom_command(
  OUTPUT ${URDF_OUTPUT}
  COMMAND ${XACRO_CMD} ${XACRO_INPUT} -o ${URDF_OUTPUT}
  DEPENDS ${XACRO_INPUT}
  COMMENT "Generating fwids_vehicle.urdf from xacro"
)

add_custom_target(generate_urdf ALL DEPENDS ${URDF_OUTPUT})

# ---- Generate ROS2 action interfaces ----------------------------------
# Skip the group membership check here (we still include the package.xml export tag),
# this avoids certain build-env ordering issues during configure.
rosidl_generate_interfaces(${PROJECT_NAME}
  SKIP_GROUP_MEMBERSHIP_CHECK
  "action/SideManoeuvre.action"
)


# install Python scripts (实车精简版)
install(PROGRAMS
  scripts/test_all_wheels.py
  scripts/side_manoeuvre_server.py
  scripts/side_manoeuvre_action_server.py
  scripts/feeder_manager.py
  scripts/omni_docking.py
  scripts/imu_converter.py
  scripts/save_map.py
  DESTINATION lib/${PROJECT_NAME}
)

# install directories (worlds 已删除，只装 launch/urdf/config)
install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
