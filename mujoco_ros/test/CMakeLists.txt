
if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)
  find_package(mujoco_ros_testing_utils REQUIRED)
  find_library(MJR_ROS1_ROSLIB_LIBRARY
    NAMES roslib
    PATHS ${catkin_LIBRARY_DIRS}
  )
  find_library(MJR_ROS1_ROSCONSOLE_LIBRARY
    NAMES rosconsole
    PATHS ${catkin_LIBRARY_DIRS}
  )
  if(NOT MJR_ROS1_ROSLIB_LIBRARY OR NOT MJR_ROS1_ROSCONSOLE_LIBRARY)
    message(FATAL_ERROR "ROS1 tests require the roslib and rosconsole libraries")
  endif()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(mujoco_ros_testing_utils REQUIRED)
endif()

# Plain (no ROS node, no launch file) gtest unit tests for the Robot
# Description Converter module -- unlike add_mujoco_ros_integration_test
# below, these compile the needed sources directly and never spin up a live node.
function(add_mujoco_ros_unit_test TEST_NAME)
  set(multiValueArgs SOURCES)
  cmake_parse_arguments(UT "" "" "${multiValueArgs}" ${ARGN})

  if(ROS_VERSION EQUAL 1)
    catkin_add_gtest(${TEST_NAME} ${UT_SOURCES})
  elseif(ROS_VERSION EQUAL 2)
    ament_add_gtest(${TEST_NAME} ${UT_SOURCES})
  endif()

  if(TARGET ${TEST_NAME})
    target_include_directories(${TEST_NAME} BEFORE PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
    )
    target_compile_definitions(${TEST_NAME} PRIVATE
      TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
    )
    target_include_directories(${TEST_NAME} PRIVATE
      ${PROJECT_SOURCE_DIR}/include
      ${GENERATED_HEADERS_DIR}
      ${mujoco_include_DIRS}
    )
    set(MJR_UNIT_LINK_MAIN TRUE)
    if(TEST_NAME STREQUAL "description_converter_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE
        ../src/extended_params.cpp
        ../src/description_converter.cpp
        ../src/mesh_uri_prep.cpp
        ../src/glb_visual_prep.cpp
        ../src/ascii_stl_convert.cpp
      )
    elseif(TEST_NAME STREQUAL "description_bundle_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/description_bundle.cpp)
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
      endif()
    elseif(TEST_NAME STREQUAL "mesh_uri_prep_test")
      # Linking the full mujoco_ros shared library pulls in runtime ROS publishers
      # that wait for a master during static initialization; this unit only needs
      # the mesh-prep sources.
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE
        ../src/mesh_uri_prep.cpp
        ../src/glb_visual_prep.cpp
        ../src/ascii_stl_convert.cpp
      )
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
      endif()
    elseif(TEST_NAME STREQUAL "glb_visual_prep_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/glb_visual_prep.cpp)
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 2 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TEST_NAME} PRIVATE -Wno-error=old-style-cast)
      endif()
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
      endif()
    elseif(TEST_NAME STREQUAL "ascii_stl_convert_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE
        ../src/mesh_uri_prep.cpp
        ../src/glb_visual_prep.cpp
        ../src/ascii_stl_convert.cpp
      )
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
      endif()
    elseif(TEST_NAME STREQUAL "viewer_geom_defaults_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/viewer_geom_defaults.cpp)
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
      elseif(ROS_VERSION EQUAL 2)
        ament_target_dependencies(${TEST_NAME}
          ament_index_cpp
          rclcpp
          rclcpp_lifecycle
          pluginlib
          tf2
          tf2_ros
          image_transport
          camera_info_manager
          geometry_msgs
          tf2_geometry_msgs
          mujoco_ros_msgs
          rosgraph_msgs
          lifecycle_msgs
          sensor_msgs
          std_srvs
        )
      endif()
    else()
      target_link_libraries(${TEST_NAME} mujoco_ros)
    endif()
    target_link_libraries(${TEST_NAME}
      mujoco::mujoco
      tinyxml2::tinyxml2
      project_option
      project_warning
    )
    if(NOT MJR_UNIT_LINK_MAIN)
      if(ROS_VERSION EQUAL 1)
        target_include_directories(${TEST_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(${TEST_NAME} ${MJR_ROS1_ROSLIB_LIBRARY})
      endif()
    endif()
    if(TEST_NAME STREQUAL "description_converter_test")
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME} gtest_main)
        target_include_directories(${TEST_NAME} SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
        target_link_libraries(${TEST_NAME}
          ${MJR_ROS1_ROSLIB_LIBRARY}
          ${MJR_ROS1_ROSCONSOLE_LIBRARY}
        )
      else()
        ament_target_dependencies(${TEST_NAME} ament_index_cpp rclcpp)
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
          target_compile_options(${TEST_NAME} PRIVATE -Wno-error=old-style-cast)
        endif()
      endif()
    endif()
    if(ROS_VERSION EQUAL 2)
      ament_target_dependencies(${TEST_NAME} ament_index_cpp rclcpp)
    endif()
    if(ROS_VERSION EQUAL 2 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND
       (TEST_NAME STREQUAL "description_converter_test" OR
        TEST_NAME STREQUAL "mesh_uri_prep_test" OR
        TEST_NAME STREQUAL "glb_visual_prep_test" OR
        TEST_NAME STREQUAL "ascii_stl_convert_test"))
      target_compile_options(${TEST_NAME} PRIVATE -Wno-error=old-style-cast)
    endif()
  endif()
endfunction()

if(ROS_VERSION EQUAL 1)
  catkin_add_gtest(tinyxml2_smoke_test tinyxml2_smoke_test.cpp)
elseif(ROS_VERSION EQUAL 2)
  ament_add_gtest(tinyxml2_smoke_test tinyxml2_smoke_test.cpp)
endif()
if(TARGET tinyxml2_smoke_test)
  if(ROS_VERSION EQUAL 1)
    target_link_libraries(tinyxml2_smoke_test gtest_main)
  endif()
  target_link_libraries(tinyxml2_smoke_test
    tinyxml2::tinyxml2
    project_option
    project_warning
  )
endif()
if(ROS_VERSION EQUAL 1)
  catkin_add_gtest(extended_params_test
    extended_params_test.cpp
    ../src/extended_params.cpp
  )
elseif(ROS_VERSION EQUAL 2)
  ament_add_gtest(extended_params_test
    extended_params_test.cpp
    ../src/extended_params.cpp
  )
endif()
if(TARGET extended_params_test)
  target_include_directories(extended_params_test BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
    ${PROJECT_SOURCE_DIR}/include
  )
  if(ROS_VERSION EQUAL 1)
    target_include_directories(extended_params_test SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  elseif(ROS_VERSION EQUAL 2)
    target_include_directories(extended_params_test PRIVATE ${CMAKE_BINARY_DIR}/include)
  endif()
  target_compile_definitions(extended_params_test PRIVATE
    TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
  )
  if(ROS_VERSION EQUAL 1)
    target_link_libraries(extended_params_test gtest_main)
  endif()
  target_link_libraries(extended_params_test
    tinyxml2::tinyxml2
    project_option
    project_warning
  )
  if(ROS_VERSION EQUAL 1)
    target_link_libraries(extended_params_test ${MJR_ROS1_ROSCONSOLE_LIBRARY})
  elseif(ROS_VERSION EQUAL 2)
    ament_target_dependencies(extended_params_test rclcpp)
  endif()
endif()
add_mujoco_ros_unit_test(description_converter_test SOURCES description_converter_test.cpp)
add_mujoco_ros_unit_test(description_bundle_test SOURCES description_bundle_test.cpp)
add_mujoco_ros_unit_test(mesh_uri_prep_test SOURCES mesh_uri_prep_test.cpp)
add_mujoco_ros_unit_test(glb_visual_prep_test SOURCES glb_visual_prep_test.cpp)
add_mujoco_ros_unit_test(ascii_stl_convert_test SOURCES ascii_stl_convert_test.cpp)
add_mujoco_ros_unit_test(viewer_geom_defaults_test SOURCES viewer_geom_defaults_test.cpp)

# Helper function handling ROS 1/2 specific setup
function(add_mujoco_ros_integration_test TEST_NAME TEST_SOURCE)
  set(options)
  set(oneValueArgs ROS1_LAUNCH ROS2_LAUNCH)
  set(multiValueArgs EXTRA_LIBS)
  cmake_parse_arguments(TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" "${ARGN}")

  if(ROS_VERSION EQUAL 1)
    add_rostest_gtest(${TEST_NAME}
      ${TEST_ROS1_LAUNCH}
      ${TEST_SOURCE}
    )

    add_dependencies(${TEST_NAME}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )

    target_link_libraries(${TEST_NAME}
      mujoco_ros
      ${mujoco_ros_testing_utils_LIBRARIES}
      project_option
      project_warning
      ${TEST_EXTRA_LIBS}
    )

    target_include_directories(${TEST_NAME}
      PRIVATE
        ${mujoco_ros_testing_utils_INCLUDE_DIRS}
    )
  elseif(ROS_VERSION EQUAL 2)
    set(TEST_EXECUTABLE ${TEST_NAME})
    set(TEST_LAUNCH_NAME ${TEST_NAME}_launch)

    ament_add_gtest_executable(${TEST_EXECUTABLE}
      ${TEST_SOURCE}
    )

    target_link_libraries(${TEST_EXECUTABLE}
      mujoco_ros
      project_option
      project_warning
      ${TEST_EXTRA_LIBS}
    )

    ament_target_dependencies(${TEST_EXECUTABLE}
      mujoco_ros_testing_utils
    )

    # Register ROS 2 test launch file
    add_launch_test(${TEST_ROS2_LAUNCH}
      TARGET ${TEST_LAUNCH_NAME}
      ARGS "test_binary:=$<TARGET_FILE:${TEST_EXECUTABLE}>"
    )
  endif()
endfunction()

add_mujoco_ros_integration_test(
  mujoco_env_test
  mujoco_env_test.cpp
  ROS1_LAUNCH launch/mujoco_env.test
  ROS2_LAUNCH launch/mujoco_env.test.py
)
target_compile_definitions(mujoco_env_test PRIVATE TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")

add_mujoco_ros_integration_test(
  description_bundle_integration_test
  description_bundle_integration_test.cpp
  ROS1_LAUNCH launch/description_bundle_integration.test
  ROS2_LAUNCH launch/description_bundle_integration.test.py
)
target_compile_definitions(description_bundle_integration_test PRIVATE
  TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
)

add_mujoco_ros_integration_test(
  ros_interface_test
  ros_interface_test.cpp
  ROS1_LAUNCH launch/ros_interface.test
  ROS2_LAUNCH launch/ros_interface.test.py
)

add_subdirectory(test_plugin)

if(ROS_VERSION EQUAL 2)
  # Forward pluginlib export vars from this directory to top-level scope.
  set(__PLUGINLIB_PLUGIN_CATEGORIES "${__PLUGINLIB_PLUGIN_CATEGORIES}" PARENT_SCOPE)
  set(__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros "${__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros}" PARENT_SCOPE)
endif()

add_mujoco_ros_integration_test(
  mujoco_ros_plugin_test
  mujoco_ros_plugin_test.cpp
  ROS1_LAUNCH launch/mujoco_ros_plugin.test
  ROS2_LAUNCH launch/mujoco_ros_plugin.test.py
  EXTRA_LIBS mujoco_testplugin
)

add_mujoco_ros_integration_test(
  mujoco_render_test
  mujoco_render_test.cpp
  ROS1_LAUNCH launch/mujoco_render.test
  ROS2_LAUNCH launch/mujoco_render.test.py
)

if(ROS_VERSION EQUAL 1)
  add_rostest(launch/python_bindings.test)
  add_rostest(launch/python_bindings_config.test)
  add_rostest(launch/python_bindings_plugin.test)
  add_rostest(launch/description_converter_bindings.test)
elseif(ROS_VERSION EQUAL 2)
  add_launch_test(launch/python_bindings.test.py
    TARGET python_bindings_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_test)
    add_dependencies(python_bindings_test pymujoco_ros)
  endif()
  add_launch_test(launch/description_converter_bindings.test.py
    TARGET description_converter_bindings_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/description_converter_bindings_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET description_converter_bindings_test)
    add_dependencies(description_converter_bindings_test pymujoco_ros)
  endif()
  add_launch_test(launch/python_bindings_config.test.py
    TARGET python_bindings_config_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_config_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_config_test)
    add_dependencies(python_bindings_config_test pymujoco_ros mujoco_testplugin)
  endif()
  add_launch_test(launch/python_bindings_plugin.test.py
    TARGET python_bindings_plugin_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_plugin_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_plugin_test)
    add_dependencies(python_bindings_plugin_test pymujoco_ros mujoco_testplugin)
  endif()
endif()
