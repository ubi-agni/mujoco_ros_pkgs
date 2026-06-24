cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros_mocap VERSION 0.10.0 LANGUAGES CXX)

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
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

  find_package(catkin REQUIRED COMPONENTS
    mujoco_ros
    mujoco_ros_msgs
    pluginlib
    roscpp
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      mujoco_ros
      mujoco_ros_msgs
      pluginlib
      roscpp
    LIBRARIES ${PROJECT_NAME}
  )

  add_library(${PROJECT_NAME}
    src/mocap_plugin.cpp
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

  install(FILES mocap_plugin.xml
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
  find_package(mujoco_ros REQUIRED)
  find_package(mujoco_ros_msgs REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  pluginlib_export_plugin_description_file(mujoco_ros mocap_plugin.ros2.xml)

  add_library(${PROJECT_NAME} SHARED
    src/mocap_plugin.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    mujoco_ros
    mujoco_ros_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(FILES mocap_plugin.ros2.xml
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY launch config assets
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_TESTING)
    add_subdirectory(test)
  endif()

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    mujoco_ros
    mujoco_ros_msgs
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  ament_package()
endif()
