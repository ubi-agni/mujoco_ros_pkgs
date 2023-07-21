# Viewer
add_library(lodepng OBJECT
  lodepng.cpp
)
set_property(TARGET lodepng PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(lodepng PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(lodepng PRIVATE project_option)
add_library(mujoco_ros::lodepng ALIAS lodepng)

add_library(platform_ui_adapter OBJECT
  $<TARGET_OBJECTS:lodepng>
  glfw_adapter.cc
  glfw_dispatch.cc
  platform_ui_adapter.cc
)
set_property(TARGET platform_ui_adapter PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(platform_ui_adapter PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${GLFW_INTERFACE_INCLUDE_DIRECTORIES}
)
target_link_libraries(platform_ui_adapter
  PUBLIC
    mujoco::mujoco
    ${GLFW}
  PRIVATE
    mujoco_ros::lodepng
    project_option
    project_warning
)
add_library(mujoco_ros::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(${PROJECT_NAME}
  $<TARGET_OBJECTS:platform_ui_adapter>
  mujoco_env.cpp
  viewer.cpp
  plugin_utils.cpp
  offscreen_camera.cpp
  offscreen_rendering.cpp
  callbacks.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC ${PROJECT_SOURCE_DIR}/include
  ${mujoco_include_DIRS}
  ${GLFW_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
   mujoco::mujoco
   mujoco_ros::platform_ui_adapter
   catkin_pkg
  PRIVATE
   mujoco_ros::lodepng
   project_option
   project_warning
)

# Node Executable
add_executable(mujoco_node
  main.cpp
)

target_link_libraries(mujoco_node
  PUBLIC
    ${PROJECT_NAME}
  PRIVATE
    project_option
    project_warning
)

# configure_coverage(TARGET ${PROJECT_NAME})
