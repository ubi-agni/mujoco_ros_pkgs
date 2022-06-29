find_package(rostest REQUIRED)

add_rostest_gtest(mujoco_single_env_test
  launch/mujoco_single_env.test
  mujoco_single_env_test.cpp
)

add_dependencies(mujoco_single_env_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_single_env_test
  ${catkin_LIBRARIES}
  mujoco_ros
)
