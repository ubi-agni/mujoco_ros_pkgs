cmake_minimum_required(VERSION 3.13)

project(mujoco_ros_control)

set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined" )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address -g -O1 -fno-inline -fno-omit-frame-pointer -fno-optimize-sibling-calls")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  controller_manager
  controller_interface
  control_toolbox
  hardware_interface
  transmission_interface
  joint_limits_interface
  angles
  urdf
  mujoco_ros
)

catkin_package(
 CATKIN_DEPENDS
  roscpp
  pluginlib
  controller_manager
  controller_interface
  control_toolbox
  hardware_interface
  transmission_interface
  joint_limits_interface
  angles
  urdf
  mujoco_ros
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} default_mujoco_ros_robot_hw_sim
)

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mujoco_ros_control_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(default_mujoco_ros_robot_hw_sim src/default_robot_hw_sim.cpp)
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
)

install(FILES
    mujoco_ros_control_plugin.xml
    default_mujoco_ros_robot_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
