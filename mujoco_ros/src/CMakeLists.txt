# Viewer
add_library(lodepng OBJECT
  ../third_party/lodepng/lodepng.cpp
)
set_property(TARGET lodepng PROPERTY POSITION_INDEPENDENT_CODE ON)
target_include_directories(lodepng SYSTEM PRIVATE
  ${PROJECT_SOURCE_DIR}/third_party/lodepng
)
target_link_libraries(lodepng PRIVATE project_option)
add_library(mujoco_ros::lodepng ALIAS lodepng)

if(WITH_GUI STREQUAL "ON")
  add_library(platform_ui_adapter OBJECT
    $<TARGET_OBJECTS:lodepng>
    glfw_adapter.cc
    platform_ui_adapter.cc
  )
  set_property(TARGET platform_ui_adapter PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_include_directories(platform_ui_adapter PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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
    runtime_options.cpp
    plugin_host.cpp
    interface.cpp
    loading.cpp
    threading.cpp
    offscreen_camera.cpp
    offscreen_rendering.cpp
    viewer.cpp
    viewer_geom_defaults.cpp
    extended_params.cpp
    description_converter.cpp
    description_bundle.cpp
    mesh_uri_prep.cpp
    glb_visual_prep.cpp
    ascii_stl_convert.cpp
  )
else()
  set(MJR_SOURCE_FILES
    main.cpp
    mujoco_env.cpp
    ros_two/ros_api.cpp
    ros_two/parameter_interface.cpp
    ros_two/plugin_utils.cpp
    physics.cpp
    runtime_options.cpp
    plugin_host.cpp
    interface.cpp
    loading.cpp
    threading.cpp
    offscreen_camera.cpp
    offscreen_rendering.cpp
    viewer.cpp
    viewer_geom_defaults.cpp
    extended_params.cpp
    description_converter.cpp
    description_bundle.cpp
    mesh_uri_prep.cpp
    glb_visual_prep.cpp
    ascii_stl_convert.cpp
  )
endif()

if(WITH_GUI STREQUAL "ON")
  list(APPEND MJR_SOURCE_FILES glfw_dispatch.cc)
endif()

add_library(${PROJECT_NAME} SHARED
  $<TARGET_OBJECTS:lodepng>
  ${MJR_SOURCE_FILES}
)

if(WITH_GUI STREQUAL "ON")
  target_sources(${PROJECT_NAME} PRIVATE $<TARGET_OBJECTS:platform_ui_adapter>)
endif()

if(WITH_GUI STREQUAL "ON")
  target_compile_definitions(${PROJECT_NAME} PUBLIC GLFW=1)
endif()
if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  target_compile_definitions(${PROJECT_NAME} PUBLIC MJR_BUILD_TESTING=1)
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${mujoco_include_DIRS}
    ${GLFW_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${GENERATED_HEADERS_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/cgltf
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/lodepng
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
   mujoco::mujoco
   mujoco_ros_render_core
  PRIVATE
   tinyxml2::tinyxml2
   project_option
   project_warning
)
if (ROS_VERSION EQUAL "1")
  target_link_libraries(${PROJECT_NAME}
    PUBLIC
      ${catkin_LIBRARIES}
    PRIVATE
      catkin_pkg
      Boost::program_options
  )

else()
  ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
      ament_index_cpp
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

if(WITH_GUI STREQUAL "ON")
  target_link_libraries(${PROJECT_NAME} PRIVATE mujoco_ros::platform_ui_adapter)
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
if (ROS_VERSION EQUAL "1")
  target_link_libraries(mujoco_node PRIVATE Boost::program_options)
endif()

if (ROS_VERSION EQUAL "2")
  target_link_libraries(${PROJECT_NAME}
    PRIVATE
      Boost::program_options
  )

  target_link_libraries(mujoco_node
    PRIVATE
      Boost::program_options
  )

endif()
