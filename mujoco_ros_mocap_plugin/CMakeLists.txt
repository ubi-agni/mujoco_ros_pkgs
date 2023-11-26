cmake_minimum_required(VERSION 3.0.2)

project(mujoco_ros_mocap)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_SANITIZE "-fsanitize=address -g -O1 -fno-inline -fno-omit-frame-pointer -fno-optimize-sibling-calls")

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros
  mujoco_ros_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  mujoco_ros
  mujoco_ros_msgs
 LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mocap_plugin.cpp
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

install(FILES
  mocap_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
