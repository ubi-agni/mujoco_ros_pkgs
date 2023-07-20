pybind_add_module(_mujoco_ros_mocap_python
  mj_ros_mocap_python.cpp
)

target_include_directories(_mujoco_ros_mocap_python SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(_mujoco_ros_mocap_python PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS _mujoco_ros_mocap_python LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
