cmake_minimum_required(VERSION 3.16)
project(mujoco_ros_render_core_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(DEFINED EXPECTED_ALTERNATE_MUJOCO_DIR AND
   NOT EXPECTED_ALTERNATE_MUJOCO_DIR STREQUAL "")
  # Exercise the installed config against caller discovery state that points
  # at a different package; the configured package root must still win.
  set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
  set(mujoco_FOUND TRUE)
  set(mujoco_DIR
    "${EXPECTED_ALTERNATE_MUJOCO_DIR}/lib/cmake/mujoco"
  )
endif()

find_package(mujoco_ros_render_core CONFIG REQUIRED)

if(NOT TARGET mujoco_ros::mujoco_ros_render_core)
  message(FATAL_ERROR "Installed RenderCore export did not provide its imported target")
endif()

function(_assert_imported_locations TARGET_NAME EXPECTED_LOCATION LABEL)
  set(_location_properties
    IMPORTED_LOCATION
    IMPORTED_LOCATION_NOCONFIG
    IMPORTED_LOCATION_DEBUG
    IMPORTED_LOCATION_RELEASE
    IMPORTED_LOCATION_RELWITHDEBINFO
    IMPORTED_LOCATION_MINSIZEREL
  )
  get_target_property(_imported_configurations
    ${TARGET_NAME} IMPORTED_CONFIGURATIONS
  )
  if(_imported_configurations AND
     NOT _imported_configurations MATCHES "-NOTFOUND$")
    foreach(_imported_configuration IN LISTS _imported_configurations)
      string(TOUPPER "${_imported_configuration}"
        _imported_configuration_upper
      )
      list(APPEND _location_properties
        "IMPORTED_LOCATION_${_imported_configuration_upper}"
      )
    endforeach()
  endif()
  list(REMOVE_DUPLICATES _location_properties)

  get_filename_component(_expected_location
    "${EXPECTED_LOCATION}" REALPATH
  )
  set(_found_location FALSE)
  foreach(_location_property IN LISTS _location_properties)
    get_target_property(_candidate_location
      ${TARGET_NAME} ${_location_property}
    )
    if(_candidate_location AND
       NOT _candidate_location MATCHES "-NOTFOUND$")
      set(_found_location TRUE)
      if(NOT EXISTS "${_candidate_location}")
        message(FATAL_ERROR
          "${LABEL} imported location ${_location_property} does not exist: "
          "${_candidate_location}"
        )
      endif()
      get_filename_component(_actual_location
        "${_candidate_location}" REALPATH
      )
      if(NOT _actual_location STREQUAL _expected_location)
        message(FATAL_ERROR
          "${LABEL} imported location ${_location_property} does not match "
          "the expected staged/configured library: ${_candidate_location}"
        )
      endif()
    endif()
  endforeach()
  if(NOT _found_location)
    message(FATAL_ERROR
      "${LABEL} imported target has no imported library location"
    )
  endif()
endfunction()

if(DEFINED INJECT_STALE_RENDER_CORE_LOCATION AND
   NOT INJECT_STALE_RENDER_CORE_LOCATION STREQUAL "")
  set_property(TARGET mujoco_ros::mujoco_ros_render_core PROPERTY
    IMPORTED_LOCATION_DEBUG "${INJECT_STALE_RENDER_CORE_LOCATION}"
  )
endif()

if(NOT DEFINED EXPECTED_RENDER_CORE_LIBRARY OR
   EXPECTED_RENDER_CORE_LIBRARY STREQUAL "")
  message(FATAL_ERROR "Installed RenderCore target location expectation is required")
endif()
_assert_imported_locations(
  mujoco_ros::mujoco_ros_render_core
  "${EXPECTED_RENDER_CORE_LIBRARY}"
  "Installed RenderCore"
)

get_target_property(_render_core_includes mujoco_ros::mujoco_ros_render_core
  INTERFACE_INCLUDE_DIRECTORIES
)
if(NOT EXPECTED_INSTALL_PREFIX STREQUAL "")
  list(FIND _render_core_includes "${EXPECTED_INSTALL_PREFIX}/include" _render_core_include_index)
  if(_render_core_include_index LESS 0)
    message(FATAL_ERROR
      "Installed RenderCore target does not expose ${EXPECTED_INSTALL_PREFIX}/include: "
      "${_render_core_includes}"
    )
  endif()
endif()

if(NOT TARGET mujoco::mujoco)
  message(FATAL_ERROR "Installed RenderCore config did not provide its MuJoCo target")
endif()

if(DEFINED INJECT_STALE_MUJOCO_LOCATION AND
   NOT INJECT_STALE_MUJOCO_LOCATION STREQUAL "")
  set_property(TARGET mujoco::mujoco PROPERTY
    IMPORTED_LOCATION_DEBUG "${INJECT_STALE_MUJOCO_LOCATION}"
  )
endif()

get_target_property(_mujoco_includes mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
if(NOT EXPECTED_MUJOCO_DIR STREQUAL "")
  list(FIND _mujoco_includes "${EXPECTED_MUJOCO_DIR}/include" _mujoco_include_index)
  if(_mujoco_include_index LESS 0)
    message(FATAL_ERROR
      "Configured MuJoCo include root was not propagated: ${_mujoco_includes}"
    )
  endif()
  get_filename_component(_expected_mujoco_library
    "${EXPECTED_MUJOCO_DIR}/lib/libmujoco.so.3.3.5" REALPATH
  )
  _assert_imported_locations(
    mujoco::mujoco
    "${_expected_mujoco_library}"
    "Configured MuJoCo"
  )
endif()

add_executable(render_core_standalone_smoke smoke.cpp)
target_link_libraries(render_core_standalone_smoke PRIVATE mujoco_ros::mujoco_ros_render_core)
if(MJR_STANDALONE_COVERAGE)
  # The installed static RenderCore archive carries GCC coverage sections.
  # Link the runtime only for the coverage smoke consumer.
  target_link_options(render_core_standalone_smoke PRIVATE --coverage)
endif()

enable_testing()
add_test(NAME disabled_backend_smoke COMMAND render_core_standalone_smoke)
