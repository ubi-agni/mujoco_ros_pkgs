cmake_minimum_required(VERSION 3.11)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Use GLVND instead of Legacy OpenGL
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

project(mujoco_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find MuJoCo
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(mujoco REQUIRED)
find_library(GLFW libglfw.so.3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    pluginlib
    mujoco_ros_msgs
    urdf
    tf2
    tf2_ros
    tf2_geometry_msgs
    image_transport
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    pluginlib
    mujoco_ros_msgs
    urdf
    tf2
    tf2_ros
    tf2_geometry_msgs
    image_transport
  INCLUDE_DIRS include ${mujoco_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${mujoco_LIBRARIES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/mujoco_env.cpp
  src/mujoco_sim.cpp
  src/uitools.c
  src/plugin_utils.cpp
  src/render_utils.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${mujoco_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GLFW}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC include
  SYSTEM PUBLIC ${catkin_INCLUDE_DIRS}
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

catkin_install_python(
  PROGRAMS
    scripts/ns_bootstrapper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
