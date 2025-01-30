cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)

find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(mujoco_ros plugins.xml)

add_library(mujoco_ros2_control SHARED src/mujoco_ros2_control.cpp)
target_compile_features(mujoco_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  mujoco_ros2_control
  rclcpp
  hardware_interface
  controller_manager
  "mujoco_ros"
  "pluginlib"
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mujoco_ros2_control
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch example
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   TARGETS mujoco_ros2_control_system
#   # EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(
#   TARGETS mujoco_ros2_control_system
#   DESTINATION lib
# )

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
  mujoco_ros2_control
  # mujoco_ros2_control_system
)

ament_export_dependencies(
  controller_manager
  hardware_interface
  mujoco_ros
  pluginlib
  rclcpp
)
# ament_export_targets(
#   export_${PROJECT_NAME}
# )

ament_package()
