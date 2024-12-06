find_package(rostest REQUIRED)
find_package(Eigen3 REQUIRED)

add_rostest_gtest(mujoco_sensors_test
  launch/mujoco_sensors.test
  mujoco_sensors_test.cpp
)

add_dependencies(mujoco_sensors_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_sensors_test
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)
