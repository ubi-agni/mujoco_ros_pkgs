find_package(rostest REQUIRED)

add_rostest_gtest(mujoco_env_test
  launch/mujoco_env.test
  mujoco_env_test.cpp
)

add_dependencies(mujoco_env_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_env_test
  ${catkin_LIBRARIES}
  mujoco_ros
)

add_rostest_gtest(ros_interface_test
  launch/ros_interface.test
  ros_interface_test.cpp
)

add_dependencies(ros_interface_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ros_interface_test
  ${catkin_LIBRARIES}
  mujoco_ros
)

add_subdirectory(test_plugin)

add_rostest_gtest(mujoco_ros_plugin_test
  launch/mujoco_ros_plugin.test
  mujoco_ros_plugin_test.cpp
)

target_link_libraries(mujoco_ros_plugin_test
  # catkin_pkg
  # project_option
  # project_warning
  GTest::gtest_main
  mujoco_ros
  mujoco_testplugin
)
