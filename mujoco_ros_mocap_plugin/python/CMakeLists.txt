pybind_add_module(pymujoco_ros_mocap
  bindings/src/pymj_ros_mocap.cpp
)

target_include_directories(pymujoco_ros_mocap SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pymujoco_ros_mocap PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS pymujoco_ros_mocap LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
