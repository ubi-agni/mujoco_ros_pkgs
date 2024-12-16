cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros VERSION 0.9.0 LANGUAGES C CXX)

#catkin_lint: ignore uninstalled_plugin

if (DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "CMAKE_BUILD_TYPE not set. Defaulting to 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

set(OpenGL_GL_PREFERENCE GLVND)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(ProjectOption)

if (ROS_VERSION EQUAL "1")
  # Find catkin macros and libraries
  # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  # is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
      roscpp
      actionlib
      pluginlib
      mujoco_ros_msgs
      urdf
      tf2
      tf2_ros
      tf2_geometry_msgs
      image_transport
      camera_info_manager
      sensor_msgs
      rosgraph_msgs
      dynamic_reconfigure
  )
else()
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(Boost REQUIRED COMPONENTS program_options)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(mujoco_ros_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
endif()

# cmake-format: off
configure_project_option(
  WARNINGS
    TARGET project_warning
  AVX
    TARGET project_option
  LINKER
    TARGET project_option
  SANITIZER
    TARGET project_option
  IPO
    DISABLE_FOR_CONFIG Debug
  # EXTRA_ARG -DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_NONE
  CATKIN_ROS
    TARGET catkin_pkg
  COMPILER_CACHE
    LAUNCHER ccache
)

# Find MuJoCo
find_package(mujoco 3.2.0 REQUIRED)

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

if (ROS_VERSION EQUAL "1")
  ## Dynamic Reconfigure
  generate_dynamic_reconfigure_options(
    cfg/SimParams.cfg
  )
endif()

if (ROS_VERSION EQUAL "1")
  # ##################################
  # # catkin specific configuration ##
  # ##################################
  # # The catkin_package macro generates cmake config files for your package
  # # Declare things to be passed to dependent projects
  # # INCLUDE_DIRS: uncomment this if your package contains header files
  # # LIBRARIES: libraries you create in this project that dependent projects also need
  # # CATKIN_DEPENDS: catkin_packages dependent projects also need
  # # DEPENDS: system dependencies of this project that dependent projects also need
  catkin_package(
    CATKIN_DEPENDS
      roscpp
      actionlib
      pluginlib
      mujoco_ros_msgs
      urdf
      tf2
      tf2_ros
      tf2_geometry_msgs
      image_transport
      camera_info_manager
      sensor_msgs
      rosgraph_msgs
      dynamic_reconfigure
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}
    DEPENDS
      mujoco
  )
endif()

if (ROS_VERSION EQUAL "1")
  # Ensure generated header path exists both with catkin and colcon
  # catkin defines CATKIN_DEVEL_PREFIX, colcon does not define it
  if(NOT DEFINED CATKIN_DEVEL_PREFIX)
    set(CATKIN_DEVEL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  set(GENERATED_HEADERS_DIR "${CATKIN_DEVEL_PREFIX}/include")
  set(GENERATED_HEADERS_INSTALL_DIR "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${PROJECT_NAME}")
else()
  set(GENERATED_HEADERS_DIR "${CMAKE_BINARY_DIR}/include")
  set(GENERATED_HEADERS_INSTALL_DIR "${CMAKE_BINARY_DIR}/include/${PROJECT_NAME}")
endif()

include(ConfigureRenderBackend)

add_subdirectory(src)

### Generate ROS version header
add_custom_command(
  OUTPUT ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.hpp always_rebuild_ros
  COMMAND ${CMAKE_COMMAND}
  -DROS_VERSION=${ROS_VERSION}
  -DGENERATED_HEADERS_DIR=${GENERATED_HEADERS_DIR}/${PROJECT_NAME}
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateRosVersionHeader.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(ros_version_h
  DEPENDS always_rebuild_ros
  COMMENT "Generating ROS version header"
)
add_dependencies(${PROJECT_NAME} ros_version_h)

if (ROS_VERSION EQUAL "1")
  # Depend on gencfg to ensure build before lib
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
endif()
# Depend on render_backend_h to ensure build before lib
add_dependencies(${PROJECT_NAME} render_backend_h)

#############
## Install ##
#############

install(FILES
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.hpp
  DESTINATION ${GENERATED_HEADERS_INSTALL_DIR}
)

if (ROS_VERSION EQUAL "1")
  install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(TARGETS ${PROJECT_NAME} mujoco_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  install(DIRECTORY launch/ros1/launch config assets
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


  # #############
  # ## Testing ##
  # #############

  # if(CATKIN_ENABLE_TESTING)
  #   add_subdirectory(test)
  # endif()
else()
  install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

  install(DIRECTORY launch/ros2/launch config assets
    DESTINATION share/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME} mujoco_node
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
  )

  ament_package()
endif()
