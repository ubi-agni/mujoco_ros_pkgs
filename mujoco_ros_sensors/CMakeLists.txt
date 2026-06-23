cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros_sensors VERSION 0.10.0 LANGUAGES CXX)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    mujoco_ros
    mujoco_ros_msgs
    pluginlib
    tf2
    tf2_geometry_msgs
    tf2_ros
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      geometry_msgs
      mujoco_ros
      mujoco_ros_msgs
      pluginlib
      tf2
      tf2_geometry_msgs
      tf2_ros
    LIBRARIES ${PROJECT_NAME}
  )

  add_library(${PROJECT_NAME}
    src/mujoco_sensor_handler_plugin.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(FILES mujoco_ros_sensors_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(DIRECTORY launch config assets
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
  endif()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(mujoco_ros REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(mujoco_ros_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  pluginlib_export_plugin_description_file(mujoco_ros mujoco_ros_sensors_plugin.ros2.xml)

  add_library(${PROJECT_NAME} SHARED
    src/mujoco_sensor_handler_plugin.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    mujoco_ros_msgs
    mujoco_ros
    pluginlib
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(DIRECTORY launch config assets
    DESTINATION share/${PROJECT_NAME}
  )

  install(FILES test/sensors_world.xml
    DESTINATION share/${PROJECT_NAME}/test
  )

  if(BUILD_TESTING)
    add_subdirectory(test)
  endif()

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    mujoco_ros
    pluginlib
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    mujoco_ros_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  ament_package()
endif()
