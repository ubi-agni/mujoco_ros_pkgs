cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(mujoco_ros2_control plugins.xml)

add_library(mujoco_ros2_control_system src/mujoco_ros2_control_system.cpp)
target_compile_features(mujoco_ros2_control_system PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mujoco_ros2_control_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  mujoco_ros2_control_system
  "mujoco_ros2_control"
  "hardware_interface"
  "rclcpp"
  "rclcpp_lifecycle"
  "pluginlib"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mujoco_ros2_control_system PRIVATE "MUJOCO_ROS2_CONTROL_SYSTEM_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mujoco_ros2_control_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

ament_export_include_directories(
  include
)
ament_export_libraries(
  mujoco_ros2_control_system
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
