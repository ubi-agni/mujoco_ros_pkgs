set(PYMUJOCO_ROS_CONTROL_SOURCES
  src/pymujoco_ros_control.cpp
)

if(ROS_VERSION EQUAL "1")
  pybind_add_module(pymujoco_ros_control ${PYMUJOCO_ROS_CONTROL_SOURCES})
else()
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
    OUTPUT_VARIABLE PYTHON_EXTENSION_SUFFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  add_library(pymujoco_ros_control MODULE ${PYMUJOCO_ROS_CONTROL_SOURCES})
  set_target_properties(pymujoco_ros_control PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_EXTENSION_SUFFIX}"
  )
endif()

if(ROS_VERSION EQUAL "1")
  target_include_directories(pymujoco_ros_control SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(pymujoco_ros_control
    PRIVATE
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )

  install(TARGETS pymujoco_ros_control
    LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
  )
else()
  target_link_libraries(pymujoco_ros_control ${PROJECT_NAME} pybind11::headers Python3::Python)
  target_include_directories(pymujoco_ros_control SYSTEM PRIVATE ${Python3_INCLUDE_DIRS})
  ament_target_dependencies(pymujoco_ros_control
    mujoco_ros
    py_binding_tools
  )

  install(TARGETS pymujoco_ros_control
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
    ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  )
endif()
