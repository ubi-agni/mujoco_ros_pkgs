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

  catkin_add_gtest(sensor_stream_test sensor_stream_test.cpp)
  target_link_libraries(sensor_stream_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(mujoco_sensors_test SYSTEM
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

  ament_add_gtest(sensor_stream_test
    sensor_stream_test.cpp
  )

  target_link_libraries(sensor_stream_test
    ${PROJECT_NAME}
  )

  ament_target_dependencies(sensor_stream_test
    rclcpp
    mujoco_ros_msgs
  )

  add_launch_test(launch/mujoco_sensors.test.py
    TARGET mujoco_sensors_test_launch
    ARGS "test_binary:=$<TARGET_FILE:mujoco_sensors_test>"
  )

  if(BUILD_PYTHON_BINDINGS)
    add_launch_test(launch/sensor_stream_python.test.py
      TARGET sensor_stream_python_test
      ARGS
        "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/test_sensor_stream_python.py"
        "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros_sensors>"
        "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
    )
  endif()
endif()
