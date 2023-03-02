cmake_minimum_required(VERSION 3.0.2)

project(mujoco_ros_control)

set( CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined" )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Disable optimization to prevent false-positives with memory analisys tools (e.g. asan or valgrind)
if (ENABLE_SANITIZER OR NO_OPTIM)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -fno-inline")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if (ENABLE_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  add_link_options(-fsanitize=address)
endif()

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
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(default_mujoco_ros_robot_hw_sim src/default_robot_hw_sim.cpp)
target_include_directories(default_mujoco_ros_robot_hw_sim SYSTEM PUBLIC
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
)

install(FILES
    mujoco_ros_control_plugin.xml
    default_mujoco_ros_robot_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
