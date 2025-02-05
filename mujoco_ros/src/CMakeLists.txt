# Viewer
add_library(lodepng OBJECT
  lodepng.cpp
)
set_property(TARGET lodepng PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(lodepng PRIVATE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(lodepng PRIVATE project_option)
add_library(mujoco_ros::lodepng ALIAS lodepng)

if(RENDERING_BACKEND STREQUAL "GLFW")
  add_library(platform_ui_adapter OBJECT
    $<TARGET_OBJECTS:lodepng>
    glfw_adapter.cc
    glfw_dispatch.cc
    platform_ui_adapter.cc
  )
  set_property(TARGET platform_ui_adapter PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_include_directories(platform_ui_adapter PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
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
endif()

if(ROS_VERSION EQUAL "1")
  set(MJR_SOURCE_FILES
    main.cpp
    mujoco_env.cpp
    ros_one/ros_api.cpp
    ros_one/parameter_interface.cpp
    ros_one/plugin_utils.cpp
    physics.cpp
    interface.cpp
    loading.cpp
    threading.cpp
    offscreen_camera.cpp
    offscreen_rendering.cpp
    viewer.cpp
  )
else()
  set(MJR_SOURCE_FILES
    main.cpp
    mujoco_env.cpp
    ros_two/ros_api.cpp
    ros_two/parameter_interface.cpp
    ros_two/plugin_utils.cpp
    physics.cpp
    interface.cpp
    loading.cpp
    threading.cpp
    offscreen_camera.cpp
    offscreen_rendering.cpp
    viewer.cpp
  )
endif()

add_library(${PROJECT_NAME} SHARED
  $<TARGET_OBJECTS:lodepng>
  ${MJR_SOURCE_FILES}
)

if(RENDERING_BACKEND STREQUAL "GLFW")
  target_sources(${PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:platform_ui_adapter>)
endif()

if (RENDERING_BACKEND)
  target_compile_definitions(${PROJECT_NAME} PUBLIC ${RENDERING_BACKEND}=1)
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${mujoco_include_DIRS}
    ${GLFW_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GENERATED_HEADERS_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
   mujoco::mujoco
  PRIVATE
   project_option
   project_warning
)

if (ROS_VERSION EQUAL "1")
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      catkin_pkg
  )

else()
  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
      rclcpp
      rclcpp_lifecycle
      pluginlib
      tf2
      tf2_ros
      image_transport
      camera_info_manager
      mujoco_ros_msgs
      rosgraph_msgs
      lifecycle_msgs
      geometry_msgs
      tf2_geometry_msgs
      sensor_msgs
      std_srvs
  )
endif()

if(RENDERING_BACKEND STREQUAL "GLFW")
  target_link_libraries(${PROJECT_NAME} PUBLIC mujoco_ros::platform_ui_adapter)
elseif(RENDERING_BACKEND STREQUAL "EGL")
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenGL::EGL)
elseif(RENDERING_BACKEND STREQUAL "OSMESA")
  target_link_libraries(${PROJECT_NAME} PUBLIC OSMesa::OSMesa)
endif()

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

if (ROS_VERSION EQUAL "2")
  target_link_libraries(mujoco_node
    PUBLIC
      Boost::program_options
  )

endif()
