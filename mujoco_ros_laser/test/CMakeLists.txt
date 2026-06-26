find_package(mujoco_ros_testing_utils REQUIRED)

if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)

  add_rostest_gtest(mujoco_ros_laser_test
    launch/mujoco_ros_laser.test
    mujoco_ros_laser_test.cpp
  )

  add_dependencies(mujoco_ros_laser_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(mujoco_ros_laser_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(mujoco_ros_laser_test
    PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${mujoco_ros_testing_utils_INCLUDE_DIRS}
  )
elseif(ROS_VERSION EQUAL 2)
  ament_add_gtest_executable(mujoco_ros_laser_test
    mujoco_ros_laser_test.cpp
  )

  target_link_libraries(mujoco_ros_laser_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(mujoco_ros_laser_test
    ament_index_cpp
    mujoco_ros_testing_utils
    rclcpp
    sensor_msgs
  )

  add_launch_test(launch/mujoco_ros_laser.test.py
    TARGET mujoco_ros_laser_test_launch
    ARGS "test_binary:=$<TARGET_FILE:mujoco_ros_laser_test>"
  )
endif()
