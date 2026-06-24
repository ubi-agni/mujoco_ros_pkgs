find_package(mujoco_ros_testing_utils REQUIRED)

if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)

  add_rostest_gtest(mujoco_ros_control_test
    launch/mujoco_ros_control.test
    mujoco_ros_control_test.cpp
  )

  if(TARGET mujoco_ros_control_test)
    add_dependencies(mujoco_ros_control_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )

    target_link_libraries(mujoco_ros_control_test
      ${PROJECT_NAME}
      default_mujoco_ros_robot_hw_sim
      ${catkin_LIBRARIES}
    )

    target_include_directories(mujoco_ros_control_test
      PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${mujoco_ros_testing_utils_INCLUDE_DIRS}
    )
  endif()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(mujoco_ros_control_test
    mujoco_ros_control_test.cpp
  )

  target_link_libraries(mujoco_ros_control_test
    ${PROJECT_NAME}
    mujoco_ros_system
  )

  ament_target_dependencies(mujoco_ros_control_test
    hardware_interface
    mujoco_ros
    mujoco_ros_testing_utils
    rclcpp
    rclcpp_lifecycle
  )

  set_tests_properties(mujoco_ros_control_test
    PROPERTIES TIMEOUT 60
  )
endif()
