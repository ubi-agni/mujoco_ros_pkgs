add_library(mujoco_ros_render_core
  ${CMAKE_CURRENT_LIST_DIR}/../src/rendering/render_demand.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/rendering/frame_boundary.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/rendering/render_backend.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/rendering/render_core.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../src/rendering/render_snapshot.cpp
)

target_include_directories(mujoco_ros_render_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mujoco_ros_render_core PUBLIC mujoco::mujoco)
target_compile_features(mujoco_ros_render_core PUBLIC cxx_std_17)
if(BUILD_TESTING OR CATKIN_ENABLE_TESTING)
  target_compile_definitions(mujoco_ros_render_core PUBLIC MJR_BUILD_TESTING=1)
endif()

if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "EGL")
  target_compile_definitions(mujoco_ros_render_core PRIVATE MJR_RENDER_BACKEND_EGL=1)
elseif(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA")
  target_compile_definitions(mujoco_ros_render_core PRIVATE MJR_RENDER_BACKEND_OSMESA=1)
endif()

if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "EGL" AND TARGET OpenGL::EGL)
  target_link_libraries(mujoco_ros_render_core PRIVATE OpenGL::EGL)
elseif(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA" AND TARGET OSMesa::OSMesa)
  target_link_libraries(mujoco_ros_render_core PRIVATE OSMesa::OSMesa)
endif()

if(PROJECT_NAME STREQUAL "mujoco_ros")
  if(ROS_VERSION EQUAL "1")
    install(TARGETS mujoco_ros_render_core
      EXPORT export_${PROJECT_NAME}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
    install(TARGETS mujoco_ros_render_core
      EXPORT mujoco_ros_render_coreTargets
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    )
  else()
    install(TARGETS mujoco_ros_render_core
      EXPORT export_${PROJECT_NAME}
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION lib
      ARCHIVE DESTINATION lib
    )
    install(TARGETS mujoco_ros_render_core
      EXPORT mujoco_ros_render_coreTargets
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION lib
      ARCHIVE DESTINATION lib
    )
  endif()

  include(CMakePackageConfigHelpers)
  set(render_core_package_dir "lib/cmake/mujoco_ros_render_core")
  configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/mujoco_ros_render_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_ros_render_coreConfig.cmake
    INSTALL_DESTINATION ${render_core_package_dir}
  )
  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_ros_render_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )
  install(EXPORT mujoco_ros_render_coreTargets
    FILE mujoco_ros_render_coreTargets.cmake
    NAMESPACE mujoco_ros::
    DESTINATION ${render_core_package_dir}
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_ros_render_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/mujoco_ros_render_coreConfigVersion.cmake
    ${CMAKE_CURRENT_LIST_DIR}/../cmake/Findmujoco.cmake
    DESTINATION ${render_core_package_dir}
  )
  if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA")
    install(FILES ${CMAKE_CURRENT_LIST_DIR}/../cmake/FindOSMesa.cmake
      DESTINATION ${render_core_package_dir}
    )
  endif()
endif()
