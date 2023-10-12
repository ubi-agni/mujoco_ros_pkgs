cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros_sensors)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Disable optimization to prevent false-positives with memory analisys tools (e.g. asan or valgrind)
if (ENABLE_SANITIZER OR NO_OPTIM)
  set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O0 -fno-inline")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

if (ENABLE_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address")
  add_link_options(-fsanitize=address)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mujoco_ros
  mujoco_ros_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  mujoco_ros
  mujoco_ros_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
 LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mujoco_sensor_handler_plugin.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
    mujoco_ros_sensors_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
