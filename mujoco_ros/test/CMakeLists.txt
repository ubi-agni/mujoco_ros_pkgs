
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
# below, these link mujoco_ros directly and never spin up a live node.
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
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 1)
        target_link_libraries(${TEST_NAME}
          ${MJR_ROS1_ROSLIB_LIBRARY}
          ${MJR_ROS1_ROSCONSOLE_LIBRARY}
        )
      elseif(ROS_VERSION EQUAL 2 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TEST_NAME} PRIVATE -Wno-error=old-style-cast)
      endif()
    elseif(TEST_NAME STREQUAL "description_bundle_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/description_bundle.cpp)
    elseif(TEST_NAME STREQUAL "mesh_uri_prep_test")
      # Keep mesh URI coverage master-free. Linking the full mujoco_ros shared
      # library pulls in runtime ROS publishers that wait for a master during
      # static initialization; this unit only needs the mesh-prep sources.
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE
        ../src/mesh_uri_prep.cpp
        ../src/glb_visual_prep.cpp
        ../src/ascii_stl_convert.cpp
      )
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
    elseif(TEST_NAME STREQUAL "glb_visual_prep_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/glb_visual_prep.cpp)
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/cgltf
      )
      if(ROS_VERSION EQUAL 2 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${TEST_NAME} PRIVATE -Wno-error=old-style-cast)
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
    elseif(TEST_NAME STREQUAL "viewer_sync_policy_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/viewer_sync_policy.cpp)
    elseif(TEST_NAME STREQUAL "viewer_geom_defaults_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/viewer_geom_defaults.cpp)
      if(ROS_VERSION EQUAL 2)
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
    elseif(TEST_NAME STREQUAL "viewer_branding_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE
        ../src/viewer_branding.cpp
        $<TARGET_OBJECTS:lodepng>
      )
      target_include_directories(${TEST_NAME} SYSTEM PRIVATE
        ${PROJECT_SOURCE_DIR}/third_party/lodepng
      )
      target_compile_definitions(${TEST_NAME} PRIVATE
        VIEWER_BRANDING_SOURCE_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets"
      )
    elseif(TEST_NAME STREQUAL "runtime_options_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/runtime_options.cpp)
    elseif(TEST_NAME STREQUAL "plugin_host_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_sources(${TEST_NAME} PRIVATE ../src/plugin_host.cpp)
    elseif(TEST_NAME STREQUAL "offscreen_transport_test")
      # Keep transport validation master-free. The production transport methods
      # remain in mujoco_ros for runtime/integration coverage; this unit uses
      # the header-only validation seam and queue/status primitives.
      set(MJR_UNIT_LINK_MAIN FALSE)
    elseif(TEST_NAME STREQUAL "render_core_consumer_test")
      set(MJR_UNIT_LINK_MAIN FALSE)
      target_link_libraries(${TEST_NAME} mujoco_ros_render_core)
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
        target_link_libraries(${TEST_NAME} gtest_main)
        target_link_libraries(${TEST_NAME} ${MJR_ROS1_ROSLIB_LIBRARY})
      endif()
    endif()
    if(ROS_VERSION EQUAL 2)
      ament_target_dependencies(${TEST_NAME} ament_index_cpp rclcpp)
    endif()
    if(ROS_VERSION EQUAL 2 AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND
       (TEST_NAME STREQUAL "mesh_uri_prep_test" OR
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
add_mujoco_ros_unit_test(viewer_sync_policy_test SOURCES viewer_sync_policy_test.cpp)
add_mujoco_ros_unit_test(viewer_geom_defaults_test SOURCES viewer_geom_defaults_test.cpp)
add_mujoco_ros_unit_test(viewer_branding_test SOURCES viewer_branding_test.cpp)
add_mujoco_ros_unit_test(runtime_options_test SOURCES runtime_options_test.cpp)
add_mujoco_ros_unit_test(plugin_host_test SOURCES plugin_host_test.cpp)
add_mujoco_ros_unit_test(offscreen_transport_test SOURCES offscreen_transport_test.cpp)
add_mujoco_ros_unit_test(render_core_consumer_test SOURCES render_core_consumer_test.cpp)
if(ROS_VERSION EQUAL 1)
  # Run under rostest so a master is available for ros::init() and the
  # /rosout publisher registration. The gtest binary is built and launched
  # by rostest, matching the mujoco_env_test convention.
  add_rostest_gtest(ros1_plugin_adapter_test
    launch/ros1_plugin_adapter.test
    ros1_plugin_adapter_test.cpp
  )
  # This unit invokes ROS 1 APIs directly, so do not rely on transitive
  # linkage from a production target it intentionally does not use.
  target_link_libraries(ros1_plugin_adapter_test
    gtest_main
    ${catkin_LIBRARIES}
    ${MJR_ROS1_ROSCONSOLE_LIBRARY}
    ${MJR_ROS1_ROSLIB_LIBRARY}
    mujoco::mujoco
    tinyxml2::tinyxml2
    project_option
    project_warning
  )
  target_include_directories(ros1_plugin_adapter_test SYSTEM PRIVATE ${catkin_INCLUDE_DIRS})
  target_include_directories(ros1_plugin_adapter_test BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
  )
  target_compile_definitions(ros1_plugin_adapter_test PRIVATE
    TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
  )
  target_include_directories(ros1_plugin_adapter_test PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${GENERATED_HEADERS_DIR}
    ${mujoco_include_DIRS}
  )
  add_dependencies(ros1_plugin_adapter_test
    ${PROJECT_NAME}_gencfg
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
endif()

function(add_render_core_unit_test TEST_NAME TEST_SOURCE)
  if(ROS_VERSION EQUAL 1)
    catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE})
  elseif(ROS_VERSION EQUAL 2)
    ament_add_gtest(${TEST_NAME} ${TEST_SOURCE})
  endif()
  if(TARGET ${TEST_NAME})
    target_include_directories(${TEST_NAME} BEFORE PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
    )
    if(ROS_VERSION EQUAL 1)
      target_link_libraries(${TEST_NAME} gtest_main)
    endif()
    target_link_libraries(${TEST_NAME}
      mujoco_ros_render_core
      project_option
      project_warning
    )
  endif()
endfunction()

add_render_core_unit_test(render_demand_test render_demand_test.cpp)
add_render_core_unit_test(frame_boundary_test frame_boundary_test.cpp)
add_render_core_unit_test(render_core_test render_core_test.cpp)
if(TARGET render_core_test)
  target_include_directories(render_core_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
endif()

add_test(NAME render_backend_config_contract
  COMMAND ${CMAKE_COMMAND}
    -P ${PROJECT_SOURCE_DIR}/cmake/render_backend_config_test.cmake
)
set_tests_properties(render_backend_config_contract PROPERTIES TIMEOUT 30)

if(WITH_GUI STREQUAL "ON")
  if(ROS_VERSION EQUAL 1)
    catkin_add_gtest(glfw_visible_gui_test
      glfw_visible_gui_test.cpp
      ../src/glfw_adapter.cc
      ../src/glfw_dispatch.cc
      ../src/platform_ui_adapter.cc
      ../src/viewer_branding.cpp
      $<TARGET_OBJECTS:lodepng>
    )
  elseif(ROS_VERSION EQUAL 2)
    ament_add_gtest(glfw_visible_gui_test
      glfw_visible_gui_test.cpp
      ../src/glfw_adapter.cc
      ../src/glfw_dispatch.cc
      ../src/platform_ui_adapter.cc
      ../src/viewer_branding.cpp
      $<TARGET_OBJECTS:lodepng>
    )
  endif()
  if(TARGET glfw_visible_gui_test)
    target_include_directories(glfw_visible_gui_test PRIVATE
      ${PROJECT_SOURCE_DIR}/include
      ${GENERATED_HEADERS_DIR}
      ${GLFW_INTERFACE_INCLUDE_DIRECTORIES}
    )
    target_include_directories(glfw_visible_gui_test SYSTEM PRIVATE
      ${PROJECT_SOURCE_DIR}/third_party/lodepng
    )
    target_compile_definitions(glfw_visible_gui_test PRIVATE
      MJR_RENDER_BACKEND_GLFW=1
    )
    if(ROS_VERSION EQUAL 1)
      target_link_libraries(glfw_visible_gui_test gtest_main)
    elseif(ROS_VERSION EQUAL 2)
      ament_target_dependencies(glfw_visible_gui_test ament_index_cpp)
    endif()
    target_link_libraries(glfw_visible_gui_test
      mujoco::mujoco
      ${GLFW}
      project_option
      project_warning
    )
  endif()

  if(ROS_VERSION EQUAL 1)
    catkin_add_gtest(glfw_adapter_failure_test
      glfw_adapter_failure_test.cpp
      ../src/glfw_adapter.cc
      ../src/platform_ui_adapter.cc
      ../src/viewer_branding.cpp
      $<TARGET_OBJECTS:lodepng>
    )
  elseif(ROS_VERSION EQUAL 2)
    ament_add_gtest(glfw_adapter_failure_test
      glfw_adapter_failure_test.cpp
      ../src/glfw_adapter.cc
      ../src/platform_ui_adapter.cc
      ../src/viewer_branding.cpp
      $<TARGET_OBJECTS:lodepng>
    )
  endif()
  if(TARGET glfw_adapter_failure_test)
    target_include_directories(glfw_adapter_failure_test PRIVATE
      ${PROJECT_SOURCE_DIR}/include
      ${GENERATED_HEADERS_DIR}
      ${GLFW_INTERFACE_INCLUDE_DIRECTORIES}
    )
    target_include_directories(glfw_adapter_failure_test SYSTEM PRIVATE
      ${PROJECT_SOURCE_DIR}/third_party/lodepng
    )
    target_compile_definitions(glfw_adapter_failure_test PRIVATE MJR_RENDER_BACKEND_GLFW=1)
    if(ROS_VERSION EQUAL 1)
      target_link_libraries(glfw_adapter_failure_test gtest_main)
    elseif(ROS_VERSION EQUAL 2)
      ament_target_dependencies(glfw_adapter_failure_test ament_index_cpp)
    endif()
    target_link_libraries(glfw_adapter_failure_test
      mujoco::mujoco
      ${GLFW}
      project_option
      project_warning
    )
  endif()
endif()

set(_render_core_standalone_coverage OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" AND
   CMAKE_CXX_FLAGS MATCHES "(^|[ ;])(--coverage|-fprofile-arcs)([ ;]|$)")
  set(_render_core_standalone_coverage ON)
endif()

add_test(NAME render_core_standalone_installed
  COMMAND ${CMAKE_COMMAND}
    -DBUILD_DIR=${CMAKE_BINARY_DIR}
    -DSTANDALONE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/render_core_standalone
    -DSTANDALONE_BINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/render_core_standalone_build
    -DINSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
    -DEXPECTED_MUJOCO_DIR=${MUJOCO_DIR}
    -DSTANDALONE_COVERAGE=${_render_core_standalone_coverage}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/render_core_standalone/run_installed_smoke.cmake
)
set_tests_properties(render_core_standalone_installed PROPERTIES RUN_SERIAL TRUE)

add_executable(render_performance_test render_performance_test.cpp)
target_link_libraries(render_performance_test
  mujoco_ros
  mujoco::mujoco
  project_option
  project_warning
)
if(ROS_VERSION EQUAL 1)
  install(TARGETS render_performance_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  target_include_directories(render_performance_test BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
    ${mujoco_ros_testing_utils_INCLUDE_DIRS}
  )
  target_link_libraries(render_performance_test ${mujoco_ros_testing_utils_LIBRARIES})
elseif(ROS_VERSION EQUAL 2)
  ament_target_dependencies(render_performance_test rclcpp sensor_msgs)
endif()
target_compile_options(render_performance_test BEFORE PRIVATE
  "-I${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include"
)

if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "EGL")
  set(RENDER_PERFORMANCE_BENCHMARK_BACKEND "EGL")
elseif(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA")
  set(RENDER_PERFORMANCE_BENCHMARK_BACKEND "OSMESA")
endif()

function(add_serial_render_performance_rostest CASE_NAME CAMERA_COUNT ITERATIONS REPEAT TIME_LIMIT)
  set(RENDER_PERFORMANCE_CASE "${CASE_NAME}")
  set(RENDER_PERFORMANCE_CAMERA_COUNT "${CAMERA_COUNT}")
  set(RENDER_PERFORMANCE_ITERATIONS "${ITERATIONS}")
  set(RENDER_PERFORMANCE_REPEAT "${REPEAT}")
  set(RENDER_PERFORMANCE_TIME_LIMIT "${TIME_LIMIT}")
  set(RENDER_PERFORMANCE_OUTPUT_DIR "${CATKIN_TEST_RESULTS_DIR}")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/launch/render_performance.test.in
    ${CMAKE_CURRENT_BINARY_DIR}/render_performance_${CASE_NAME}.test
    @ONLY
  )
  add_rostest(
    ${CMAKE_CURRENT_BINARY_DIR}/render_performance_${CASE_NAME}.test
    DEPENDENCIES render_performance_test
  )
  # add_rostest derives this component from the generated file's path.
  set(_render_performance_test_name
    "test_render_performance_${CASE_NAME}.test"
  )
  set(_render_performance_run_target
    "run_tests_${PROJECT_NAME}_rostest_${_render_performance_test_name}"
  )
  set(_render_performance_hidden_run_target
    "_run_tests_${PROJECT_NAME}_rostest_${_render_performance_test_name}"
  )
  # DEFINED guard first: an empty ${...} leaves a dangling TARGET clause
  # that fails CMake if() parsing on the first call (no previous case yet).
  if(DEFINED _previous_render_performance_run_target
     AND DEFINED _previous_render_performance_hidden_run_target)
  if(TARGET ${_previous_render_performance_run_target} AND
     TARGET ${_render_performance_run_target} AND
     TARGET ${_previous_render_performance_hidden_run_target} AND
     TARGET ${_render_performance_hidden_run_target})
    add_dependencies(
      ${_render_performance_run_target}
      ${_previous_render_performance_run_target}
    )
    add_dependencies(
      ${_render_performance_hidden_run_target}
      ${_previous_render_performance_hidden_run_target}
    )
  endif()
  endif()
  set(_previous_render_performance_run_target
    "${_render_performance_run_target}"
    PARENT_SCOPE
  )
  set(_previous_render_performance_hidden_run_target
    "${_render_performance_hidden_run_target}"
    PARENT_SCOPE
  )
  set_tests_properties(
    "_ctest_${PROJECT_NAME}_rostest_${_render_performance_test_name}"
    PROPERTIES RUN_SERIAL TRUE
  )
endfunction()

if(ROS_VERSION EQUAL 1 AND RENDER_PERFORMANCE_BENCHMARK_BACKEND)
  # Each case gets its own rostest process. RUN_SERIAL covers CTest; the
  # visible and hidden catkin target chains cover both aggregate run paths.
  add_serial_render_performance_rostest(completion 4 2 1 30.0)
  add_serial_render_performance_rostest(attribution 2 3 2 60.0)
  add_serial_render_performance_rostest(exit_deadline 4 10 2 180.0)
  add_serial_render_performance_rostest(camera_1 1 2 1 30.0)
  add_serial_render_performance_rostest(camera_2 2 2 1 30.0)
  add_serial_render_performance_rostest(camera_4 4 2 1 30.0)
endif()

if(WITH_GUI STREQUAL "ON")
  add_executable(viewer_connection_benchmark viewer_connection_benchmark.cpp)
  target_link_libraries(viewer_connection_benchmark
    mujoco_ros
    mujoco::mujoco
    project_option
    project_warning
  )
  target_include_directories(viewer_connection_benchmark BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
  )
  if(ROS_VERSION EQUAL 1)
    target_link_libraries(viewer_connection_benchmark ${mujoco_ros_testing_utils_LIBRARIES})
  elseif(ROS_VERSION EQUAL 2)
    ament_target_dependencies(viewer_connection_benchmark rclcpp)
  endif()
endif()
# Build the harness binary for the advisory workflow / local campaigns only.
# Do not register render_performance_launch_test (or rostest) in the regular
# package suite — that CTest would fail the matrix. OSMesa advisory CI in
# .github/workflows/ci.render-performance.yaml is the only CI runner.

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_test(NAME render_performance_report
  COMMAND ${Python3_EXECUTABLE} -m unittest -v test_render_performance_report.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Helper function handling ROS 1/2 specific setup
function(add_mujoco_ros_integration_test TEST_NAME TEST_SOURCE)
  set(options USE_GTEST_MAIN)
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

    if(TEST_USE_GTEST_MAIN)
      target_link_libraries(${TEST_NAME} gtest_main)
    endif()

    target_link_libraries(${TEST_NAME}
      mujoco_ros
      ${mujoco_ros_testing_utils_LIBRARIES}
      project_option
      project_warning
      ${TEST_EXTRA_LIBS}
    )

    target_include_directories(${TEST_NAME}
      BEFORE
      PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
        ${mujoco_ros_testing_utils_INCLUDE_DIRS}
    )
    target_compile_options(${TEST_NAME} PRIVATE
      "-I${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include"
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

    target_include_directories(${TEST_EXECUTABLE} BEFORE PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include
    )
    target_compile_options(${TEST_EXECUTABLE} BEFORE PRIVATE
      "-I${CMAKE_CURRENT_SOURCE_DIR}/../../mujoco_ros_testing_utils/include"
    )

    # Register ROS 2 test launch file
    add_launch_test(${TEST_ROS2_LAUNCH}
      TARGET ${TEST_LAUNCH_NAME}
      ARGS "test_binary:=$<TARGET_FILE:${TEST_EXECUTABLE}>"
    )
  endif()
endfunction()

add_mujoco_ros_integration_test(
  description_converter_integration_test
  description_converter_test.cpp
  USE_GTEST_MAIN
  ROS1_LAUNCH launch/description_converter_integration.test
  ROS2_LAUNCH launch/description_converter_integration.test.py
)
target_compile_definitions(description_converter_integration_test PRIVATE
  TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
  DESCRIPTION_CONVERTER_INTEGRATION_TESTS
)
add_mujoco_ros_integration_test(
  mujoco_env_test
  mujoco_env_test.cpp
  ROS1_LAUNCH launch/mujoco_env.test
  ROS2_LAUNCH launch/mujoco_env.test.py
)
target_compile_definitions(mujoco_env_test PRIVATE TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources")
if(ROS_VERSION EQUAL 1 AND WITH_GUI STREQUAL "ON")
  # Child-process lifetime coverage has separate rostest ownership so its
  # _exit()-based child status cannot obscure regular GUI environment results.
  add_rostest(launch/mujoco_env_lifetime.test)
endif()

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
  add_rostest(launch/python_bindings_policy.test)
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
  add_launch_test(launch/python_bindings.test.py
    TARGET python_bindings_reload_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_reload_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_reload_test)
    add_dependencies(python_bindings_reload_test pymujoco_ros)
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
  add_launch_test(launch/python_bindings_config.test.py
    TARGET python_bindings_policy_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_policy_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_policy_test)
    add_dependencies(python_bindings_policy_test pymujoco_ros)
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
