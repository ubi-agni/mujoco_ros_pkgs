cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros VERSION 1.0.0 LANGUAGES C CXX)

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

option(BUILD_PYTHON_BINDINGS "Build the pybind11 Python bindings" ON)

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

  if(BUILD_PYTHON_BINDINGS)
    find_package(py_binding_tools QUIET)
    if(NOT py_binding_tools_FOUND)
      message(FATAL_ERROR
        "py_binding_tools was not found. Install it, or disable the Python "
        "bindings by configuring with -DBUILD_PYTHON_BINDINGS=OFF.")
    endif()
    find_package(pybind11_catkin REQUIRED)
  endif()

  # Find MuJoCo
  find_package(mujoco 3.3.5 REQUIRED)
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)

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
  find_package(Boost REQUIRED COMPONENTS program_options)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(camera_info_manager REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(mujoco_ros_msgs REQUIRED)
  find_package(rosgraph_msgs REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  include(InstallMujocoToWorkspace)

  if(NOT TARGET mujoco::mujoco)
    add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION "${MUJOCO_DIR}/lib/libmujoco.so"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
    )
  endif()
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
  catkin_python_setup()

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
  set(GENERATED_HEADERS_INSTALL_DIR "${CATKIN_PACKAGE_INCLUDE_DESTINATION}")
else()
  set(GENERATED_HEADERS_DIR "${CMAKE_BINARY_DIR}/include")
  set(GENERATED_HEADERS_INSTALL_DIR "include/${PROJECT_NAME}")
endif()

include(ConfigureRenderBackend)

add_subdirectory(src)

### Generate configured headers
file(MAKE_DIRECTORY ${GENERATED_HEADERS_DIR}/${PROJECT_NAME})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/header_templates/ros_version.hpp.in
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.hpp
)

find_package(Git QUIET)
set(MJR_GIT_DESCRIBE "unknown")
set(MJR_GIT_COMMIT "unknown")
set(MJR_GIT_BRANCH "unknown")
set(MJR_GIT_DIRTY 0)
if(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../.git")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --always
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE MJR_GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE MJR_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE MJR_GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} diff --quiet
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE MJR_GIT_DIFF_RESULT
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} diff --cached --quiet
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE MJR_GIT_DIFF_CACHED_RESULT
    ERROR_QUIET
  )
  if(MJR_GIT_DIFF_RESULT OR MJR_GIT_DIFF_CACHED_RESULT)
    set(MJR_GIT_DIRTY 1)
  endif()
endif()
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/header_templates/version.hpp.in
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/version.hpp
)

if (ROS_VERSION EQUAL "1")
  # Depend on gencfg to ensure build before lib
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
endif()

if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(python/bindings)
endif()

#############
## Install ##
#############

install(FILES
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.hpp
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/version.hpp
  DESTINATION ${GENERATED_HEADERS_INSTALL_DIR}
)

if (ROS_VERSION EQUAL "1")
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(TARGETS ${PROJECT_NAME} mujoco_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

  install(FILES launch/launch_server.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY config assets
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  catkin_install_python(PROGRAMS
    test/python/python_bindings_test.py
    test/python/python_bindings_config_test.py
    test/python/python_bindings_plugin_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


  # #############
  # ## Testing ##
  # #############

  if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
  endif()
else()

  if(BUILD_TESTING)
    add_subdirectory(test)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    # TODO: Remove after ROS 1 and 2 builds are unified
    # Disable uncrustify and flake for now to avoid too much deviation from the noetic-devel branch
    set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_export_include_directories(
    include
    ${GENERATED_HEADERS_DIR}
    ${mujoco_INCLUDE_DIRS}
  )

  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    ament_cmake
    ament_cmake_python
    rclcpp
    rclcpp_lifecycle
    pluginlib
    tf2
    tf2_ros
    image_transport
    camera_info_manager
    geometry_msgs
    tf2_geometry_msgs
    mujoco_ros_msgs
    rosgraph_msgs
    lifecycle_msgs
    sensor_msgs
    std_srvs
    mujoco
  )
  if(BUILD_PYTHON_BINDINGS)
    ament_export_dependencies(py_binding_tools pybind11_vendor)
  endif()

  install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
  )

  # Explicitly copy the generated header files
  install(FILES
    ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/ros_version.hpp
    ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/render_backend.hpp
    ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/version.hpp
    DESTINATION include/${PROJECT_NAME}
  )

  install(DIRECTORY launch/ros2/launch config assets
    DESTINATION share/${PROJECT_NAME}
  )

  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/src/${PROJECT_NAME})

  install(PROGRAMS
    test/python/python_bindings_test.py
    test/python/python_bindings_config_test.py
    test/python/python_bindings_plugin_test.py
    DESTINATION lib/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    ARCHIVE DESTINATION lib
  )

  install(FILES
    cmake/mujoco_ros-extras.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  install(TARGETS mujoco_node
    LIBRARY DESTINATION lib/${PROJECT_NAME}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib/${PROJECT_NAME}
  )

  ament_package(
    CONFIG_EXTRAS cmake/mujoco_ros-extras.cmake
  )
endif()
