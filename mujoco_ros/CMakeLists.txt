cmake_minimum_required(VERSION 3.4)
project(mujoco_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    pluginlib
    cmake_modules
    mujoco_ros_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pluginlib
    mujoco_ros_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/mujoco_env.cpp
  src/mujoco_sim.cpp
  src/uitools.c
  src/plugin_utils.cpp
)

find_library(GLFW libglfw.so.3)
LINK_DIRECTORIES( ${MUJOCO_HOME}/mujoco/bin )
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   mujoco
   GL
   GLEW
   ${GLFW}
 )
 target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
 )
 add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
 )

add_executable(mujoco_server
  src/main.cpp
)
target_link_libraries(mujoco_server
  ${PROJECT_NAME}
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} mujoco_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#   add_subdirectory(test)
# endif()
