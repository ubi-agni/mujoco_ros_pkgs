cmake_minimum_required(VERSION 3.8)
project(mujoco_ros2_dummy_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

pluginlib_export_plugin_description_file(mujoco_ros plugins.xml)

add_library(dummy_ros2_plugin SHARED src/dummy_ros2_plugin.cpp)
target_compile_features(dummy_ros2_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(dummy_ros2_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  dummy_ros2_plugin
  rclcpp
  std_srvs
  std_msgs
  "mujoco_ros"
  "pluginlib"
)

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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS dummy_ros2_plugin
  # EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  dummy_ros2_plugin
)
ament_export_dependencies(
  mujoco_ros
  pluginlib
  rclcpp
  std_msgs
  std_srvs
)

ament_package()
