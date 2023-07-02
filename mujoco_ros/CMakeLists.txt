cmake_minimum_required(VERSION 3.11)
include(CMakePrintHelpers)

cmake_print_variables(ENABLE_SANITIZER)
cmake_print_variables(NO_OPTIM)

# Disable optimization to prevent false-positives with memory analisys tools
if (NOT ENABLE_SANITIZER AND NOT NO_OPTIM)
  # INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
  set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
endif()

# Use GLVND instead of Legacy OpenGL
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

project(mujoco_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXTRA_COMPILE_OPTIONS
  -Werror
  -Wall
  -Wimplicit-fallthrough
  -Wunused
  -Wno-int-in-bool-context
  -Wno-sign-compare
  -Wno-unknown-pragmas
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

# Disable optimization to prevent false-positives with memory analisys tools (e.g. asan or valgrind)
if (ENABLE_SANITIZER OR NO_OPTIM)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -fno-inline")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

cmake_print_variables(CMAKE_CXX_FLAGS)
cmake_print_variables(CMAKE_BUILD_TYPE)

if (ENABLE_SANITIZER)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
  add_link_options(-fsanitize=address)
endif()


# Find MuJoCo
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(mujoco 2.3.6 REQUIRED)
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
    camera_info_manager
    sensor_msgs
    pybind11_catkin
)

catkin_python_setup()

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
    camera_info_manager
    sensor_msgs
  INCLUDE_DIRS include ${mujoco_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} ${mujoco_LIBRARIES}
)

###########
## Build ##
###########

pybind_add_module(pymujoco_ros
  python/pymujoco_ros.cpp
)

add_library(platform_ui_adapter OBJECT
  src/glfw_adapter.cc
  src/glfw_dispatch.cc
  src/platform_ui_adapter.cc
)
set_property(TARGET platform_ui_adapter PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(platform_ui_adapter PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${GLFW_INTERFACE_INCLUDE_DIRECTORIES}
)

target_link_libraries(platform_ui_adapter PUBLIC mujoco::mujoco ${GLFW})
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(${PROJECT_NAME}
  $<TARGET_OBJECTS:platform_ui_adapter>
  src/mujoco_env.cpp
  src/viewer.cpp
  src/lodepng.cpp
  src/plugin_utils.cpp
  src/offscreen_camera.cpp
  src/offscreen_rendering.cpp
  src/callbacks.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  ${mujoco_INCLUDE_DIRS}
  ${GLFW_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    mujoco::mujoco
    mujoco::platform_ui_adapter
    ${catkin_LIBRARIES}
)

add_executable(mujoco_node
  src/main.cpp
)

target_include_directories(mujoco_node
  SYSTEM PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mujoco_node
  PUBLIC
   ${PROJECT_NAME}
  #  mujoco::platform_ui_adapter
  #  ${catkin_LIBRARIES}
)

target_include_directories(pymujoco_ros SYSTEM PRIVATE include ${mujoco_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(pymujoco_ros PRIVATE
  ${PACKAGE_NAME}
#   ${catkin_LIBRARIES}
#   mujoco::platform_ui_adapter
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} mujoco_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS pymujoco_ros LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
