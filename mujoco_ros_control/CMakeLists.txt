cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros_control VERSION 1.0.0 LANGUAGES CXX)

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

set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address -g -O1 -fno-inline -fno-omit-frame-pointer -fno-optimize-sibling-calls")

option(BUILD_PYTHON_BINDINGS "Build the pybind11 Python bindings" ON)

if(ROS_VERSION EQUAL 1)
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

  find_package(catkin REQUIRED COMPONENTS
    angles
    controller_interface
    controller_manager
    control_toolbox
    hardware_interface
    joint_limits_interface
    mujoco_ros
    pluginlib
    roscpp
    transmission_interface
    urdf
  )

  if(BUILD_PYTHON_BINDINGS)
    find_package(py_binding_tools QUIET)
    if(NOT py_binding_tools_FOUND)
      message(FATAL_ERROR
        "py_binding_tools was not found. Install it, or disable the Python "
        "bindings by configuring with -DBUILD_PYTHON_BINDINGS=OFF.")
    endif()
    find_package(pybind11_catkin REQUIRED)
  endif()

  catkin_python_setup()

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      angles
      controller_interface
      controller_manager
      control_toolbox
      hardware_interface
      joint_limits_interface
      mujoco_ros
      pluginlib
      roscpp
      transmission_interface
      urdf
    LIBRARIES ${PROJECT_NAME} default_mujoco_ros_robot_hw_sim
  )

  add_library(${PROJECT_NAME}
    src/ros_one/mujoco_ros_control_plugin.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  add_library(default_mujoco_ros_robot_hw_sim
    src/ros_one/default_robot_hw_sim.cpp
  )
  target_include_directories(default_mujoco_ros_robot_hw_sim PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(default_mujoco_ros_robot_hw_sim
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME} default_mujoco_ros_robot_hw_sim
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
  )

  install(FILES
      mujoco_ros_control_plugin.xml
      default_mujoco_ros_robot_hw_sim_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(DIRECTORY launch example
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    OPTIONAL
  )

  if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python/bindings)
  endif()

  if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
  endif()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(mujoco_ros REQUIRED)
  if(BUILD_PYTHON_BINDINGS)
    find_package(py_binding_tools QUIET)
    if(NOT py_binding_tools_FOUND)
      message(FATAL_ERROR
        "py_binding_tools was not found. Install it, or disable the Python "
        "bindings by configuring with -DBUILD_PYTHON_BINDINGS=OFF.")
    endif()
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
  endif()
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

  pluginlib_export_plugin_description_file(mujoco_ros mujoco_ros_control_plugin.ros2.xml)
  pluginlib_export_plugin_description_file(mujoco_ros_control mujoco_ros_system_plugin.ros2.xml)

  add_library(${PROJECT_NAME} SHARED
    src/ros_two/mujoco_ros_control.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}
    controller_manager
    hardware_interface
    lifecycle_msgs
    mujoco_ros
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  add_library(mujoco_ros_system SHARED
    src/ros_two/mujoco_ros_system.cpp
  )
  target_include_directories(mujoco_ros_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(mujoco_ros_system
    hardware_interface
    lifecycle_msgs
    mujoco_ros
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(TARGETS ${PROJECT_NAME} mujoco_ros_system
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(FILES
      mujoco_ros_control_plugin.ros2.xml
      mujoco_ros_system_plugin.ros2.xml
    DESTINATION share/${PROJECT_NAME}
  )

  install(DIRECTORY launch example
    DESTINATION share/${PROJECT_NAME}
  )

  if(BUILD_PYTHON_BINDINGS)
    add_subdirectory(python/bindings)
  endif()
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/src/${PROJECT_NAME})

  if(BUILD_TESTING)
    add_subdirectory(test)
  endif()

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    controller_manager
    hardware_interface
    lifecycle_msgs
    mujoco_ros
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
  if(BUILD_PYTHON_BINDINGS)
    ament_export_dependencies(py_binding_tools pybind11_vendor)
  endif()

  ament_package()
endif()
