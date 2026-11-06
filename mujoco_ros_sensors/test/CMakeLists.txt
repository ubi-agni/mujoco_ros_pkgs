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

  # Master-free unit: compile stream + hub sources only. Linking ${PROJECT_NAME}
  # pulls libmujoco_ros / plugin NodeHandle paths that advertise against a ROS
  # master and hang with Connection refused when none is running.
  find_library(MJR_ROS1_ROSLIB_LIBRARY NAMES roslib PATHS ${catkin_LIBRARY_DIRS})
  find_library(MJR_ROS1_ROSCONSOLE_LIBRARY NAMES rosconsole PATHS ${catkin_LIBRARY_DIRS})
  find_library(MJR_ROS1_ROSCPP_LIBRARY NAMES roscpp PATHS ${catkin_LIBRARY_DIRS})
  if(NOT MJR_ROS1_ROSLIB_LIBRARY OR NOT MJR_ROS1_ROSCONSOLE_LIBRARY OR NOT MJR_ROS1_ROSCPP_LIBRARY)
    message(FATAL_ERROR "sensor_stream_test (ROS1) requires roslib, rosconsole, and roscpp")
  endif()
  catkin_add_gtest(sensor_stream_test
    sensor_stream_test.cpp
    ../src/native_sensor_streams.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros/src/sensor_hub.cpp
  )
  target_include_directories(sensor_stream_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )
  target_include_directories(sensor_stream_test SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(sensor_stream_test
    gtest_main
    mujoco::mujoco
    ${MJR_ROS1_ROSLIB_LIBRARY}
    ${MJR_ROS1_ROSCONSOLE_LIBRARY}
    ${MJR_ROS1_ROSCPP_LIBRARY}
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
