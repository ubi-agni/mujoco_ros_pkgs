find_package(Eigen3 REQUIRED)
find_package(mujoco_ros_testing_utils REQUIRED)

if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)

  add_rostest_gtest(mujoco_sensors_test
    launch/mujoco_sensors.test
    mujoco_sensors_test.cpp
  )

  add_dependencies(mujoco_sensors_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(mujoco_sensors_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    Eigen3::Eigen
  )

  target_include_directories(mujoco_sensors_test
    PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${mujoco_ros_testing_utils_INCLUDE_DIRS}
  )
elseif(ROS_VERSION EQUAL 2)
  ament_add_gtest_executable(mujoco_sensors_test
    mujoco_sensors_test.cpp
  )

  target_link_libraries(mujoco_sensors_test
    ${PROJECT_NAME}
    Eigen3::Eigen
  )

  ament_target_dependencies(mujoco_sensors_test
    ament_index_cpp
    geometry_msgs
    mujoco_ros_testing_utils
    mujoco_ros_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
  )

  add_launch_test(launch/mujoco_sensors.test.py
    TARGET mujoco_sensors_test_launch
    TIMEOUT 90
    ARGS "test_binary:=$<TARGET_FILE:mujoco_sensors_test>"
  )
endif()
