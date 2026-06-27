set(PYMUJOCO_ROS_SOURCES
  src/env_structs.cpp
  src/mujoco_env.cpp
  src/module.cpp
  src/plugins.cpp
  src/rendering.cpp
)

if(ROS_VERSION EQUAL "1")
  pybind_add_module(pymujoco_ros ${PYMUJOCO_ROS_SOURCES})
else()
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
    OUTPUT_VARIABLE PYTHON_EXTENSION_SUFFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  add_library(pymujoco_ros MODULE ${PYMUJOCO_ROS_SOURCES})
  set_target_properties(pymujoco_ros PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_EXTENSION_SUFFIX}"
  )
endif()

target_include_directories(pymujoco_ros
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(ROS_VERSION EQUAL "1")
  target_include_directories(pymujoco_ros SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(pymujoco_ros
    PRIVATE
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )

  install(TARGETS pymujoco_ros
    LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
  )
else()
  target_link_libraries(pymujoco_ros ${PROJECT_NAME} pybind11::headers Python3::Python)
  target_include_directories(pymujoco_ros SYSTEM PRIVATE ${Python3_INCLUDE_DIRS})
  ament_target_dependencies(pymujoco_ros
    rclcpp
    py_binding_tools
    sensor_msgs
  )

  install(TARGETS pymujoco_ros
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
    ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  )
endif()
