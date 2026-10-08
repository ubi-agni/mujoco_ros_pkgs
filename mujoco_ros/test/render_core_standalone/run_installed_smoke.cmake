if(NOT DEFINED BUILD_DIR OR NOT DEFINED STANDALONE_SOURCE_DIR OR
   NOT DEFINED STANDALONE_BINARY_DIR OR NOT DEFINED INSTALL_PREFIX OR
   NOT DEFINED EXPECTED_MUJOCO_DIR)
  message(FATAL_ERROR "standalone smoke build, source, install, and MuJoCo paths are required")
endif()

if(EXPECTED_MUJOCO_DIR STREQUAL "")
  message(FATAL_ERROR "standalone smoke requires the configured MuJoCo root")
endif()

set(_staged_install_prefix "${STANDALONE_BINARY_DIR}/staged_install")
file(REMOVE_RECURSE "${_staged_install_prefix}")
file(MAKE_DIRECTORY "${_staged_install_prefix}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${BUILD_DIR}" --parallel 2
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_output
  ERROR_VARIABLE build_error
)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "package build failed before staging install (${build_result}):\n"
    "${build_output}\n${build_error}"
  )
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}"
    --prefix "${_staged_install_prefix}"
  RESULT_VARIABLE install_result
  OUTPUT_VARIABLE install_output
  ERROR_VARIABLE install_error
)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR
    "fresh staged install failed (${install_result}):\n"
    "${install_output}\n${install_error}"
  )
endif()

set(_render_core_config
  "${_staged_install_prefix}/lib/cmake/mujoco_ros_render_core/mujoco_ros_render_coreConfig.cmake"
)
# ament symlink-install rewrites install(FILES|TARGETS|DIRECTORY) to the
# package's original install space and ignores `cmake --install --prefix`.
# MuJoCo is staged via install(CODE) (honors --prefix); overlay the rest from
# the real package install so the consumer still sees one self-contained prefix.
if(NOT EXISTS "${_render_core_config}")
  if(NOT EXISTS "${INSTALL_PREFIX}/lib/cmake/mujoco_ros_render_core/mujoco_ros_render_coreConfig.cmake")
    message(FATAL_ERROR
      "installed RenderCore config missing from package install space: "
      "${INSTALL_PREFIX}/lib/cmake/mujoco_ros_render_core/mujoco_ros_render_coreConfig.cmake"
    )
  endif()
  file(COPY "${INSTALL_PREFIX}/" DESTINATION "${_staged_install_prefix}")
endif()
if(NOT EXISTS "${_render_core_config}")
  message(FATAL_ERROR "installed RenderCore config missing after install: ${_render_core_config}")
endif()
foreach(_header IN ITEMS
    "${_staged_install_prefix}/include/mujoco/mujoco.h"
    "${_staged_install_prefix}/include/mujoco_ros/rendering/frame_boundary.hpp"
    "${_staged_install_prefix}/include/mujoco_ros/rendering/render_core.hpp"
)
  if(NOT EXISTS "${_header}")
    message(FATAL_ERROR "installed header missing after install: ${_header}")
  endif()
endforeach()
if(NOT EXISTS "${EXPECTED_MUJOCO_DIR}/lib/libmujoco.so.3.3.5")
  message(FATAL_ERROR
    "configured MuJoCo library missing: ${EXPECTED_MUJOCO_DIR}/lib/libmujoco.so.3.3.5"
  )
endif()

get_filename_component(_mujoco_module_dir
  "${STANDALONE_SOURCE_DIR}/../../cmake" ABSOLUTE
)
get_filename_component(_expected_mujoco_library
  "${EXPECTED_MUJOCO_DIR}/lib/libmujoco.so.3.3.5" REALPATH
)
set(_default_discovery_source
  "${STANDALONE_BINARY_DIR}/default_mujoco_discovery"
)
set(_default_discovery_binary
  "${STANDALONE_BINARY_DIR}/default_mujoco_discovery_build"
)
file(REMOVE_RECURSE
  "${_default_discovery_source}"
  "${_default_discovery_binary}"
)
file(MAKE_DIRECTORY "${_default_discovery_source}")
file(WRITE
  "${_default_discovery_source}/CMakeLists.txt"
  [=[
cmake_minimum_required(VERSION 3.16)
project(mujoco_default_discovery LANGUAGES NONE)

list(PREPEND CMAKE_MODULE_PATH "@MUJOCO_MODULE_DIR@")
find_package(mujoco 3.3.5 REQUIRED)

if(DEFINED MUJOCO_DIR AND NOT MUJOCO_DIR STREQUAL "")
  message(FATAL_ERROR "Default MuJoCo discovery unexpectedly configured MUJOCO_DIR")
endif()
if(NOT TARGET mujoco::mujoco)
  message(FATAL_ERROR "Default MuJoCo discovery did not provide its imported target")
endif()
get_target_property(_mujoco_includes mujoco::mujoco
  INTERFACE_INCLUDE_DIRECTORIES
)
list(FIND _mujoco_includes "@MUJOCO_INCLUDE_DIR@" _mujoco_include_index)
if(_mujoco_include_index LESS 0)
  message(FATAL_ERROR
    "Default MuJoCo discovery selected the wrong include root: "
    "${_mujoco_includes}"
  )
endif()
get_target_property(_mujoco_location mujoco::mujoco IMPORTED_LOCATION)
get_filename_component(_mujoco_location
  "${_mujoco_location}" REALPATH
)
if(NOT _mujoco_location STREQUAL "@MUJOCO_LIBRARY@")
  message(FATAL_ERROR
    "Default MuJoCo discovery selected the wrong library: ${_mujoco_location}"
  )
endif()
]=]
)
file(READ
  "${_default_discovery_source}/CMakeLists.txt"
  _default_discovery_contents
)
string(REPLACE "@MUJOCO_MODULE_DIR@" "${_mujoco_module_dir}"
  _default_discovery_contents "${_default_discovery_contents}"
)
string(REPLACE "@MUJOCO_INCLUDE_DIR@" "${EXPECTED_MUJOCO_DIR}/include"
  _default_discovery_contents "${_default_discovery_contents}"
)
string(REPLACE "@MUJOCO_LIBRARY@" "${_expected_mujoco_library}"
  _default_discovery_contents "${_default_discovery_contents}"
)
file(WRITE
  "${_default_discovery_source}/CMakeLists.txt"
  "${_default_discovery_contents}"
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env
          --unset=MUJOCO_DIR
          --unset=mujoco_DIR
          --unset=CMAKE_PREFIX_PATH
          "${CMAKE_COMMAND}"
          -S "${_default_discovery_source}"
          -B "${_default_discovery_binary}"
          "-DCMAKE_INCLUDE_PATH=${EXPECTED_MUJOCO_DIR}/include"
          "-DCMAKE_LIBRARY_PATH=${EXPECTED_MUJOCO_DIR}/lib"
  RESULT_VARIABLE default_discovery_result
  OUTPUT_VARIABLE default_discovery_output
  ERROR_VARIABLE default_discovery_error
)
if(NOT default_discovery_result EQUAL 0)
  message(FATAL_ERROR
    "default MuJoCo discovery regression failed (${default_discovery_result}):\n"
    "${default_discovery_output}\n${default_discovery_error}"
  )
endif()

file(GLOB _render_core_libraries
  LIST_DIRECTORIES false
  "${_staged_install_prefix}/lib/libmujoco_ros_render_core.so*"
  "${_staged_install_prefix}/lib/libmujoco_ros_render_core.a"
)
if(NOT _render_core_libraries)
  message(FATAL_ERROR
    "staged RenderCore library missing under ${_staged_install_prefix}/lib"
  )
endif()
set(_expected_render_core_library
  "${_staged_install_prefix}/lib/libmujoco_ros_render_core.so"
)
if(NOT EXISTS "${_expected_render_core_library}")
  set(_expected_render_core_library
    "${_staged_install_prefix}/lib/libmujoco_ros_render_core.a"
  )
endif()

set(_alternate_mujoco_prefix "${STANDALONE_BINARY_DIR}/alternate_mujoco")
file(REMOVE_RECURSE "${_alternate_mujoco_prefix}")
file(MAKE_DIRECTORY "${_alternate_mujoco_prefix}/lib/cmake/mujoco")
file(MAKE_DIRECTORY "${_alternate_mujoco_prefix}/lib")
file(WRITE
  "${_alternate_mujoco_prefix}/lib/cmake/mujoco/mujocoConfigVersion.cmake"
  [=[
set(PACKAGE_VERSION "3.3.5")
if(PACKAGE_FIND_VERSION VERSION_GREATER PACKAGE_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_FIND_VERSION VERSION_EQUAL PACKAGE_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()
]=]
)
file(WRITE
  "${_alternate_mujoco_prefix}/lib/cmake/mujoco/mujocoConfig.cmake"
  "set(mujoco_FOUND TRUE)\n"
  "set(mujoco_VERSION 3.3.5)\n"
  "message(FATAL_ERROR \"stale alternate MuJoCo config was selected\")\n"
)
file(WRITE
  "${_alternate_mujoco_prefix}/lib/cmake/mujoco/Findmujoco.cmake"
  "message(FATAL_ERROR \"stale alternate MuJoCo module was selected\")\n"
)
file(WRITE
  "${_alternate_mujoco_prefix}/lib/libstale_mujoco.so"
  "stale alternate library\n"
)

set(_standalone_coverage_arg)
if(STANDALONE_COVERAGE)
  set(_standalone_coverage_arg -DMJR_STANDALONE_COVERAGE=ON)
endif()

set(_standalone_consumer_binary_dir "${STANDALONE_BINARY_DIR}/consumer")
file(REMOVE_RECURSE "${_standalone_consumer_binary_dir}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env
          --unset=ROS_VERSION
          --unset=MUJOCO_DIR
          --unset=mujoco_DIR
          --unset=mujoco_ros_render_core_DIR
          --unset=CMAKE_PREFIX_PATH
          "${CMAKE_COMMAND}"
          -S "${STANDALONE_SOURCE_DIR}"
          -B "${_standalone_consumer_binary_dir}"
          "-DCMAKE_PREFIX_PATH=${_alternate_mujoco_prefix};${_staged_install_prefix}"
          "-DCMAKE_MODULE_PATH=${_alternate_mujoco_prefix}/lib/cmake/mujoco"
          "-DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON"
          "-DEXPECTED_INSTALL_PREFIX=${_staged_install_prefix}"
          "-DEXPECTED_RENDER_CORE_LIBRARY=${_expected_render_core_library}"
          "-DEXPECTED_MUJOCO_DIR=${EXPECTED_MUJOCO_DIR}"
          "-DEXPECTED_ALTERNATE_MUJOCO_DIR=${_alternate_mujoco_prefix}"
          ${_standalone_coverage_arg}
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_output
  ERROR_VARIABLE configure_error
)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR
    "installed RenderCore standalone configure failed (${configure_result}):\n"
    "${configure_output}\n${configure_error}"
  )
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${_standalone_consumer_binary_dir}" --parallel 2
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_output
  ERROR_VARIABLE build_error
)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "installed RenderCore standalone build failed (${build_result}):\n"
    "${build_output}\n${build_error}"
  )
endif()

execute_process(
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${_standalone_consumer_binary_dir}" --output-on-failure
  RESULT_VARIABLE test_result
  OUTPUT_VARIABLE test_output
  ERROR_VARIABLE test_error
)
if(NOT test_result EQUAL 0)
  message(FATAL_ERROR
    "installed RenderCore downstream smoke failed (${test_result}):\n"
    "${test_output}\n${test_error}"
  )
endif()

function(_assert_stale_location_rejected VARIABLE_NAME)
  set(_negative_binary_dir
    "${STANDALONE_BINARY_DIR}/${VARIABLE_NAME}_consumer"
  )
  file(REMOVE_RECURSE "${_negative_binary_dir}")
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env
            --unset=ROS_VERSION
            --unset=MUJOCO_DIR
            --unset=mujoco_DIR
            --unset=mujoco_ros_render_core_DIR
            --unset=CMAKE_PREFIX_PATH
            "${CMAKE_COMMAND}"
            -S "${STANDALONE_SOURCE_DIR}"
            -B "${_negative_binary_dir}"
            "-DCMAKE_PREFIX_PATH=${_alternate_mujoco_prefix};${_staged_install_prefix}"
            "-DCMAKE_MODULE_PATH=${_alternate_mujoco_prefix}/lib/cmake/mujoco"
            "-DCMAKE_FIND_PACKAGE_PREFER_CONFIG=ON"
            "-DEXPECTED_INSTALL_PREFIX=${_staged_install_prefix}"
            "-DEXPECTED_RENDER_CORE_LIBRARY=${_expected_render_core_library}"
            "-DEXPECTED_MUJOCO_DIR=${EXPECTED_MUJOCO_DIR}"
            "-DEXPECTED_ALTERNATE_MUJOCO_DIR=${_alternate_mujoco_prefix}"
            "-D${VARIABLE_NAME}=${_alternate_mujoco_prefix}/lib/libstale_mujoco.so"
    RESULT_VARIABLE _negative_result
    OUTPUT_VARIABLE _negative_output
    ERROR_VARIABLE _negative_error
  )
  if(_negative_result EQUAL 0)
    message(FATAL_ERROR
      "stale imported location was accepted for ${VARIABLE_NAME}"
    )
  endif()
  set(_negative_diagnostics "${_negative_output}\n${_negative_error}")
  if(NOT _negative_diagnostics MATCHES
     "IMPORTED_LOCATION_DEBUG")
    message(FATAL_ERROR
      "stale imported location rejection for ${VARIABLE_NAME} lacked the "
      "configuration-specific diagnostic:\n${_negative_diagnostics}"
    )
  endif()
endfunction()

_assert_stale_location_rejected(INJECT_STALE_RENDER_CORE_LOCATION)
_assert_stale_location_rejected(INJECT_STALE_MUJOCO_LOCATION)
