
file(GLOB PYTHON_SOURCES
  "*.cpp"
)

message(WARN "PY-SOURCES: ${PYTHON_SOURCES}")

pybind_add_module(_mujoco_ros_python
  ${PYTHON_SOURCES}
)

target_include_directories(_mujoco_ros_python SYSTEM PRIVATE
  ${mujoco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(_mujoco_ros_python PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS _mujoco_ros_python LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
