set(PYMUJOCO_ROS_LASER_SOURCES
  src/pymujoco_ros_laser.cpp
)

if(ROS_VERSION EQUAL "1")
  pybind_add_module(pymujoco_ros_laser ${PYMUJOCO_ROS_LASER_SOURCES})
else()
  execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '.so')"
    OUTPUT_VARIABLE PYTHON_EXTENSION_SUFFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  add_library(pymujoco_ros_laser MODULE ${PYMUJOCO_ROS_LASER_SOURCES})
  set_target_properties(pymujoco_ros_laser PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_EXTENSION_SUFFIX}"
  )
endif()

if(ROS_VERSION EQUAL "1")
  target_include_directories(pymujoco_ros_laser SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(pymujoco_ros_laser
    PRIVATE
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
  )

  install(TARGETS pymujoco_ros_laser
    LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION}
  )
else()
  target_link_libraries(pymujoco_ros_laser ${PROJECT_NAME} pybind11::headers Python3::Python)
  target_include_directories(pymujoco_ros_laser SYSTEM PRIVATE ${Python3_INCLUDE_DIRS})
  ament_target_dependencies(pymujoco_ros_laser
    mujoco_ros
    mujoco_ros_sensors
    py_binding_tools
  )

  install(TARGETS pymujoco_ros_laser
    LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
    RUNTIME DESTINATION ${PYTHON_INSTALL_DIR}
    ARCHIVE DESTINATION ${PYTHON_INSTALL_DIR}
  )
endif()
