find_package(rostest REQUIRED)

add_rostest_gtest(mujoco_mocap_test
  launch/mujoco_mocap_test.test
  mujoco_mocap_test.cpp
)

add_dependencies(mujoco_mocap_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_mocap_test
  ${catkin_LIBRARIES}
  mujoco_mocap
)
