cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco_ros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

pluginlib_export_plugin_description_file(mujoco_ros mujoco_ros2_sensors_plugin.xml)

set(PLUGIN_NAME "mujoco_ros2_sensors_plugin")

add_library(${PLUGIN_NAME} SHARED src/mujoco_sensor_handler_plugin.cpp)
target_compile_features(${PLUGIN_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PLUGIN_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PLUGIN_NAME}
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  mujoco_ros_msgs
  sensor_msgs
  "mujoco_ros"
  "pluginlib"
  tf2
  tf2_ros
  tf2_geometry_msgs
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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PLUGIN_NAME}
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PLUGIN_NAME}
)
ament_export_dependencies(
  mujoco_ros
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  mujoco_ros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
