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
  mujoco_ros
  project_option
  project_warning
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
  mujoco_ros
  project_option
  project_warning
)

add_subdirectory(test_plugin)

add_rostest_gtest(mujoco_ros_plugin_test
  launch/mujoco_ros_plugin.test
  mujoco_ros_plugin_test.cpp
)

add_dependencies(mujoco_ros_plugin_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_ros_plugin_test
  mujoco_ros
  mujoco_testplugin
  project_option
  project_warning
)

add_rostest_gtest(mujoco_render_test
  launch/mujoco_render.test
  mujoco_render_test.cpp
)

add_dependencies(mujoco_render_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_render_test
  mujoco_ros
  project_option
  project_warning
)

install(FILES
  empty_world.xml
  equality_world.xml
  camera_world.xml
  pendulum_world.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
