cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros_laser VERSION 0.10.0 LANGUAGES CXX)

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
    mujoco_ros_sensors
    py_binding_tools
    pybind11_catkin
    pluginlib
    roscpp
    sensor_msgs
  )

  catkin_python_setup()

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      mujoco_ros
      mujoco_ros_sensors
      py_binding_tools
      pluginlib
      roscpp
      sensor_msgs
    LIBRARIES ${PROJECT_NAME}
  )

  add_library(${PROJECT_NAME}
    src/laser.cpp
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

  install(FILES mujoco_laser_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(DIRECTORY launch config assets
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  add_subdirectory(python/bindings)

  if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
  endif()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(mujoco_ros REQUIRED)
  find_package(mujoco_ros_sensors REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(py_binding_tools REQUIRED)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  if(NOT PYTHON_INSTALL_DIR)
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -c "import os, sysconfig; print(sysconfig.get_path('platlib', vars={'base': '', 'platbase': ''}).lstrip(os.sep))"
      OUTPUT_VARIABLE PYTHON_INSTALL_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(sensor_msgs REQUIRED)

  pluginlib_export_plugin_description_file(mujoco_ros mujoco_laser_plugin.ros2.xml)

  add_library(${PROJECT_NAME} SHARED
    src/laser.cpp
  )

  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}
    mujoco_ros
    mujoco_ros_sensors
    pluginlib
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
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

  install(FILES mujoco_laser_plugin.ros2.xml
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY launch config assets
    DESTINATION share/${PROJECT_NAME}
  )

  add_subdirectory(python/bindings)
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/src/${PROJECT_NAME})

  if(BUILD_TESTING)
    add_subdirectory(test)
  endif()

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    mujoco_ros
    mujoco_ros_sensors
    py_binding_tools
    pybind11_vendor
    pluginlib
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
  )

  ament_package()
endif()
