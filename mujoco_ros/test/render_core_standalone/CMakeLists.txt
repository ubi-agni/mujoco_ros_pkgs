cmake_minimum_required(VERSION 3.16)
project(mujoco_ros_render_core_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mujoco_ros_render_core CONFIG REQUIRED)

add_executable(render_core_standalone_smoke smoke.cpp)
target_link_libraries(render_core_standalone_smoke PRIVATE mujoco_ros::mujoco_ros_render_core)
if(MJR_STANDALONE_COVERAGE)
  # The installed static RenderCore archive carries GCC coverage sections.
  # Link the runtime only for the coverage smoke consumer.
  target_link_options(render_core_standalone_smoke PRIVATE --coverage)
endif()

enable_testing()
add_test(NAME disabled_backend_smoke COMMAND render_core_standalone_smoke)
