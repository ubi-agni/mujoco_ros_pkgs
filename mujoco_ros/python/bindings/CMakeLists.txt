
file(GLOB PYTHON_SOURCES
  "src/*.cpp"
)

message(WARN "PY-SOURCES: ${PYTHON_SOURCES}")

pybind_add_module(pymujoco_ros
  ${PYTHON_SOURCES}
)

target_include_directories(pymujoco_ros SYSTEM PRIVATE
  ${mujoco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(pymujoco_ros PRIVATE
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS pymujoco_ros LIBRARY DESTINATION ${CATKIN_GLOBAL_PYTHON_DESTINATION})
