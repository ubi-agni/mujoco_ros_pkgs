find_package(mujoco_ros_testing_utils REQUIRED)

if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)

  add_rostest_gtest(mocap_test
    launch/mocap_test.test
    mocap_test.cpp
  )

  add_dependencies(mocap_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )

  target_link_libraries(mocap_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(mocap_test
    PRIVATE
      ${catkin_INCLUDE_DIRS}
      ${mujoco_ros_testing_utils_INCLUDE_DIRS}
  )
elseif(ROS_VERSION EQUAL 2)
  ament_add_gtest_executable(mocap_test
    mocap_test.cpp
  )

  target_link_libraries(mocap_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(mocap_test
    ament_index_cpp
    mujoco_ros
    mujoco_ros_msgs
    mujoco_ros_testing_utils
    rclcpp
  )

  add_launch_test(launch/mocap_test.test.py
    TARGET mocap_test_launch
    TIMEOUT 45
    ARGS "test_binary:=$<TARGET_FILE:mocap_test>"
  )
endif()
