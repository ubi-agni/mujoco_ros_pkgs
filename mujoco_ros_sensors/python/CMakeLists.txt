pybind_add_module(pymujoco_ros_sensors
  bindings/src/pymj_ros_sensors.cpp
)

target_include_directories(pymujoco_ros_sensors SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pymujoco_ros_sensors PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS pymujoco_ros_sensors LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
