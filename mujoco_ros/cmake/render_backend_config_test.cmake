get_filename_component(_repo_root "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(_fixture_binary_root "${_repo_root}/build/render_backend_config_test")
file(MAKE_DIRECTORY "${_fixture_binary_root}")

set(_active_cmake_files
  "${_repo_root}/mujoco_ros/cmake/ConfigureRenderBackend.cmake"
  "${_repo_root}/mujoco_ros/src/CMakeLists.txt"
  "${_repo_root}/mujoco_ros/test/CMakeLists.txt"
  "${_repo_root}/mujoco_ros/render_core/CMakeLists.txt"
  "${_repo_root}/mujoco_ros/cmake/mujoco_ros-extras.cmake"
  "${_repo_root}/mujoco_ros/cmake/mujoco_ros-extras.installspace.in"
  "${_repo_root}/mujoco_ros/cmake/mujoco_ros-extras.develspace.in"
  "${_repo_root}/mujoco_ros/render_core/mujoco_ros_render_coreConfig.cmake.in"
  "${_repo_root}/mujoco_ros/test/render_core_standalone/run_installed_smoke.cmake"
)
foreach(_active_file IN LISTS _active_cmake_files)
  if(NOT EXISTS "${_active_file}")
    message(FATAL_ERROR "Active CMake file missing for leak scan: ${_active_file}")
  endif()
  file(READ "${_active_file}" _active_contents)
  if(_active_contents MATCHES "RENDERING_BACKEND|OFFSCREEN_RENDERING_BACKEND")
    message(FATAL_ERROR
      "Legacy derived CMake identifier leaked in active file: ${_active_file}")
  endif()
endforeach()

# Remove fixture artifacts previously written to the repository root by cmake -P runs.
foreach(_artifact IN ITEMS
    render_backend_config_fixture
    render_backend_config_gui_on_offscreen_any
    render_backend_config_gui_on_offscreen_egl
    render_backend_config_gui_on_offscreen_osmesa
    render_backend_config_gui_on_offscreen_disable
    render_backend_config_gui_off_offscreen_any
    render_backend_config_gui_off_offscreen_egl
    render_backend_config_gui_off_offscreen_osmesa
    render_backend_config_gui_off_offscreen_disable
    render_backend_config_invalid_gui
    render_backend_config_invalid_offscreen
    render_backend_config_old_render_backend_name
    render_backend_config_old_offscreen_render_backend_name
    render_backend_config_gui_on_no_glfw
  )
  if(EXISTS "${_repo_root}/${_artifact}")
    file(REMOVE_RECURSE "${_repo_root}/${_artifact}")
  endif()
endforeach()

set(_fixture_source "${_fixture_binary_root}/fixture")
file(MAKE_DIRECTORY "${_fixture_source}")
file(WRITE "${_fixture_source}/CMakeLists.txt" [=[
cmake_minimum_required(VERSION 3.16)
project(render_backend_config_test LANGUAGES NONE)

set(PROJECT_NAME render_backend_config_test)
set(GENERATED_HEADERS_DIR "${CMAKE_BINARY_DIR}/generated")
set(GENERATED_HEADERS_INSTALL_DIR include)

if(MOCK_GLFW)
  set(_MUJOCO_RENDER_TEST_MOCK_GLFW "${MOCK_GLFW}")
endif()
if(MOCK_NO_GLFW)
  set(_MUJOCO_RENDER_TEST_MOCK_NO_GLFW TRUE)
endif()
if(MOCK_EGL)
  set(_MUJOCO_RENDER_TEST_MOCK_EGL TRUE)
endif()
if(MOCK_OSMESA)
  set(_MUJOCO_RENDER_TEST_MOCK_OSMESA TRUE)
endif()

if(CASE STREQUAL "gui_on_offscreen_any")
  set(WITH_GUI ON CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND ANY CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_on_offscreen_egl")
  set(WITH_GUI ON CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND EGL CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_on_offscreen_osmesa")
  set(WITH_GUI ON CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND OSMESA CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_on_offscreen_disable")
  set(WITH_GUI ON CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND DISABLE CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_off_offscreen_any")
  set(WITH_GUI OFF CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND ANY CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_off_offscreen_egl")
  set(WITH_GUI OFF CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND EGL CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_off_offscreen_osmesa")
  set(WITH_GUI OFF CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND OSMESA CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_off_offscreen_disable")
  set(WITH_GUI OFF CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND DISABLE CACHE STRING "" FORCE)
elseif(CASE STREQUAL "gui_on_no_glfw")
  set(WITH_GUI ON CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND DISABLE CACHE STRING "" FORCE)
elseif(CASE STREQUAL "invalid_gui")
  set(WITH_GUI BOGUS CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND DISABLE CACHE STRING "" FORCE)
elseif(CASE STREQUAL "invalid_offscreen")
  set(WITH_GUI OFF CACHE STRING "" FORCE)
  set(OFFSCREEN_BACKEND GLFW CACHE STRING "" FORCE)
elseif(CASE STREQUAL "old_render_backend_name")
  set(RENDER_BACKEND GLFW CACHE STRING "" FORCE)
elseif(CASE STREQUAL "old_offscreen_render_backend_name")
  set(OFFSCREEN_RENDER_BACKEND OSMESA CACHE STRING "" FORCE)
else()
  message(FATAL_ERROR "Unknown backend configuration test case: ${CASE}")
endif()

include("@CONFIGURE_SCRIPT@")

file(WRITE "${CMAKE_BINARY_DIR}/render-backend-result.txt"
  "visible=${_MUJOCO_VISIBLE_RENDER_BACKEND}\noffscreen=${_MUJOCO_RESOLVED_OFFSCREEN_BACKEND}\n")

if(CASE STREQUAL "gui_on_offscreen_disable")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "GLFW")
    message(FATAL_ERROR "WITH_GUI=ON must enable GLFW when GLFW is mocked available.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=DISABLE must disable offscreen RenderCore.")
  endif()
elseif(CASE STREQUAL "gui_off_offscreen_disable")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "WITH_GUI=OFF must disable visible GUI.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=DISABLE must disable offscreen RenderCore.")
  endif()
elseif(CASE STREQUAL "gui_off_offscreen_egl")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "WITH_GUI=OFF must not enable visible GLFW.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "EGL")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=EGL must remain independent of WITH_GUI=OFF.")
  endif()
elseif(CASE STREQUAL "gui_off_offscreen_osmesa")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "WITH_GUI=OFF must not enable visible GLFW.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=OSMESA must remain independent of WITH_GUI=OFF.")
  endif()
elseif(CASE STREQUAL "gui_on_offscreen_egl")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "GLFW")
    message(FATAL_ERROR "WITH_GUI=ON must enable GLFW when GLFW is mocked available.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "EGL")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=EGL must remain independent of WITH_GUI=ON.")
  endif()
elseif(CASE STREQUAL "gui_on_offscreen_osmesa")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "GLFW")
    message(FATAL_ERROR "WITH_GUI=ON must enable GLFW when GLFW is mocked available.")
  endif()
  if(NOT _MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "OSMESA")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=OSMESA must remain independent of WITH_GUI=ON.")
  endif()
elseif(CASE STREQUAL "gui_on_offscreen_any")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "GLFW")
    message(FATAL_ERROR "WITH_GUI=ON must enable GLFW when GLFW is mocked available.")
  endif()
  if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=ANY must resolve to a mocked offscreen backend.")
  endif()
elseif(CASE STREQUAL "gui_off_offscreen_any")
  if(NOT _MUJOCO_VISIBLE_RENDER_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "WITH_GUI=OFF must disable visible GUI.")
  endif()
  if(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND STREQUAL "NO")
    message(FATAL_ERROR "OFFSCREEN_BACKEND=ANY must resolve to a mocked offscreen backend.")
  endif()
endif()
]=])

file(READ "${_fixture_source}/CMakeLists.txt" _fixture_contents)
string(REPLACE "@CONFIGURE_SCRIPT@" "${CMAKE_CURRENT_LIST_DIR}/ConfigureRenderBackend.cmake"
  _fixture_contents "${_fixture_contents}")
file(WRITE "${_fixture_source}/CMakeLists.txt" "${_fixture_contents}")

function(_configure_case NAME)
  set(_binary_dir "${_fixture_binary_root}/cases/${NAME}")
  file(REMOVE_RECURSE "${_binary_dir}")
  set(_mock_args)
  if(ARGC GREATER 1)
    foreach(_mock_arg IN LISTS ARGN)
      list(APPEND _mock_args "${_mock_arg}")
    endforeach()
  endif()
  execute_process(
    COMMAND ${CMAKE_COMMAND}
      -S "${_fixture_source}"
      -B "${_binary_dir}"
      -DCASE=${NAME}
      ${_mock_args}
    RESULT_VARIABLE _result
    OUTPUT_VARIABLE _output
    ERROR_VARIABLE _error
  )
  set(_configure_case_result "${_result}" PARENT_SCOPE)
  set(_configure_case_output "${_output}\n${_error}" PARENT_SCOPE)
  set(_configure_case_binary_dir "${_binary_dir}" PARENT_SCOPE)
endfunction()

function(_assert_header_macros CASE_NAME BINARY_DIR EXPECTED_VISIBLE EXPECTED_OFFSCREEN)
  set(_header_path "${BINARY_DIR}/generated/render_backend_config_test/render_backend.hpp")
  if(NOT EXISTS "${_header_path}")
    message(FATAL_ERROR
      "Case '${CASE_NAME}' did not generate render_backend.hpp at '${_header_path}'.")
  endif()
  file(READ "${_header_path}" _header_contents)
  if(NOT _header_contents MATCHES "#define RENDER_BACKEND ${EXPECTED_VISIBLE}_BACKEND")
    message(FATAL_ERROR
      "Case '${CASE_NAME}' generated header missing visible macro "
      "RENDER_BACKEND ${EXPECTED_VISIBLE}_BACKEND:\n${_header_contents}")
  endif()
  if(NOT _header_contents MATCHES "#define OFFSCREEN_RENDER_BACKEND ${EXPECTED_OFFSCREEN}_BACKEND")
    message(FATAL_ERROR
      "Case '${CASE_NAME}' generated header missing offscreen macro "
      "OFFSCREEN_RENDER_BACKEND ${EXPECTED_OFFSCREEN}_BACKEND:\n${_header_contents}")
  endif()
endfunction()

function(run_case NAME EXPECTED_RESULT EXPECTED_VISIBLE EXPECTED_OFFSCREEN)
  _configure_case(${NAME} ${ARGN})
  if(EXPECTED_RESULT STREQUAL "success")
    if(_configure_case_result)
      message(FATAL_ERROR
        "Valid independent backend configuration '${NAME}' failed:\n"
        "${_configure_case_output}")
    endif()
    file(READ "${_configure_case_binary_dir}/render-backend-result.txt" _result_contents)
    if(NOT _result_contents MATCHES "visible=${EXPECTED_VISIBLE}")
      message(FATAL_ERROR
        "Case '${NAME}' resolved visible backend to '${_result_contents}', expected visible=${EXPECTED_VISIBLE}.")
    endif()
    if(NOT _result_contents MATCHES "offscreen=${EXPECTED_OFFSCREEN}")
      message(FATAL_ERROR
        "Case '${NAME}' resolved offscreen backend to '${_result_contents}', expected offscreen=${EXPECTED_OFFSCREEN}.")
    endif()
    _assert_header_macros(${NAME} "${_configure_case_binary_dir}"
      ${EXPECTED_VISIBLE} ${EXPECTED_OFFSCREEN})
  elseif(EXPECTED_RESULT STREQUAL "failure")
    if(NOT _configure_case_result)
      message(FATAL_ERROR "Invalid backend configuration was accepted: ${NAME}")
    endif()
  elseif(EXPECTED_RESULT STREQUAL "glfw_required_failure")
    if(NOT _configure_case_result)
      message(FATAL_ERROR "WITH_GUI=ON without GLFW was accepted: ${NAME}")
    endif()
    if(NOT _configure_case_output MATCHES "WITH_GUI=ON requires GLFW")
      message(FATAL_ERROR
        "Case '${NAME}' failed without GLFW requirement message:\n${_configure_case_output}")
    endif()
  elseif(EXPECTED_RESULT STREQUAL "migration_failure")
    if(NOT _configure_case_result)
      message(FATAL_ERROR "Removed selector name was accepted: ${NAME}")
    endif()
    if(NOT _configure_case_output MATCHES "WITH_GUI")
      message(FATAL_ERROR
        "Case '${NAME}' failed without naming WITH_GUI migration target:\n${_configure_case_output}")
    endif()
  elseif(EXPECTED_RESULT STREQUAL "offscreen_migration_failure")
    if(NOT _configure_case_result)
      message(FATAL_ERROR "Removed selector name was accepted: ${NAME}")
    endif()
    if(NOT _configure_case_output MATCHES "OFFSCREEN_BACKEND")
      message(FATAL_ERROR
        "Case '${NAME}' failed without naming OFFSCREEN_BACKEND migration target:\n${_configure_case_output}")
    endif()
  else()
    message(FATAL_ERROR "Unknown expected result: ${EXPECTED_RESULT}")
  endif()
endfunction()

set(_mock_glfw -DMOCK_GLFW=/mock/libglfw.so)
set(_mock_egl -DMOCK_EGL=ON)
set(_mock_osmesa -DMOCK_OSMESA=ON)

run_case(gui_on_offscreen_any success GLFW EGL ${_mock_glfw} ${_mock_egl})
run_case(gui_on_offscreen_egl success GLFW EGL ${_mock_glfw} ${_mock_egl})
run_case(gui_on_offscreen_osmesa success GLFW OSMESA ${_mock_glfw} ${_mock_osmesa})
run_case(gui_on_offscreen_disable success GLFW NO ${_mock_glfw})
run_case(gui_off_offscreen_any success NO EGL ${_mock_egl})
run_case(gui_off_offscreen_egl success NO EGL ${_mock_egl})
run_case(gui_off_offscreen_osmesa success NO OSMESA ${_mock_osmesa})
run_case(gui_off_offscreen_disable success NO NO)

run_case(gui_on_no_glfw glfw_required_failure "" "" -DMOCK_NO_GLFW=ON)
run_case(invalid_gui failure "" "")
run_case(invalid_offscreen failure "" "")
run_case(old_render_backend_name migration_failure "" "")
run_case(old_offscreen_render_backend_name offscreen_migration_failure "" "")
