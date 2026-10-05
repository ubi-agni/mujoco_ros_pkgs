include_guard()

# Reject removed public cache variables (no compatibility aliases).
if(DEFINED CACHE{RENDER_BACKEND})
  message(FATAL_ERROR
    "RENDER_BACKEND was removed. Configure visible GUI with WITH_GUI=ON or WITH_GUI=OFF.")
endif()
if(DEFINED CACHE{OFFSCREEN_RENDER_BACKEND})
  message(FATAL_ERROR
    "OFFSCREEN_RENDER_BACKEND was removed. Configure offscreen RenderCore with "
    "OFFSCREEN_BACKEND=ANY, EGL, OSMESA, or DISABLE.")
endif()

set(WITH_GUI "ON" CACHE STRING "Build visible GLFW GUI")
set_property(CACHE WITH_GUI PROPERTY STRINGS "ON" "OFF")
set(OFFSCREEN_BACKEND "ANY" CACHE STRING "Choose offscreen RenderCore backend")
set_property(CACHE OFFSCREEN_BACKEND PROPERTY STRINGS "ANY" "EGL" "OSMESA" "DISABLE")

if(NOT WITH_GUI STREQUAL "ON" AND NOT WITH_GUI STREQUAL "OFF")
  message(FATAL_ERROR "Unknown WITH_GUI='${WITH_GUI}'. Choose ON or OFF.")
endif()

if(NOT OFFSCREEN_BACKEND STREQUAL "ANY" AND
    NOT OFFSCREEN_BACKEND STREQUAL "EGL" AND
    NOT OFFSCREEN_BACKEND STREQUAL "OSMESA" AND
    NOT OFFSCREEN_BACKEND STREQUAL "DISABLE")
  message(FATAL_ERROR
    "Unknown OFFSCREEN_BACKEND='${OFFSCREEN_BACKEND}'. "
    "Choose ANY, EGL, OSMESA, or DISABLE.")
endif()

if(DEFINED _MUJOCO_RENDER_TEST_MOCK_NO_GLFW AND _MUJOCO_RENDER_TEST_MOCK_NO_GLFW)
  set(GLFW "")
elseif(DEFINED _MUJOCO_RENDER_TEST_MOCK_GLFW)
  set(GLFW "${_MUJOCO_RENDER_TEST_MOCK_GLFW}")
else()
  find_library(GLFW libglfw.so.3) # Visible GUI only.
endif()

set(_MUJOCO_VISIBLE_RENDER_BACKEND "NO")
if(WITH_GUI STREQUAL "ON")
  if(GLFW)
    set(_MUJOCO_VISIBLE_RENDER_BACKEND "GLFW")
    message(STATUS "GLFW3 found. Visible GUI available.")
  else()
    message(FATAL_ERROR
      "WITH_GUI=ON requires GLFW3 (libglfw.so.3). Install GLFW or configure with WITH_GUI=OFF.")
  endif()
endif()

set(_offscreen_request "${OFFSCREEN_BACKEND}")

set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "NO")
if(_offscreen_request STREQUAL "DISABLE")
  message(STATUS "Offscreen RenderCore disabled.")
elseif(_offscreen_request STREQUAL "EGL")
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_EGL AND _MUJOCO_RENDER_TEST_MOCK_EGL)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "EGL selected for offscreen RenderCore.")
  else()
    find_package(OpenGL COMPONENTS OpenGL EGL REQUIRED)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "EGL selected for offscreen RenderCore.")
  endif()
elseif(_offscreen_request STREQUAL "OSMESA")
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_OSMESA AND _MUJOCO_RENDER_TEST_MOCK_OSMESA)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
    message(STATUS "OSMesa selected for offscreen RenderCore.")
  else()
    find_package(OSMesa REQUIRED)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
    message(STATUS "OSMesa selected for offscreen RenderCore.")
  endif()
else()
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_EGL AND _MUJOCO_RENDER_TEST_MOCK_EGL)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "EGL found. Selected for offscreen RenderCore.")
  elseif(DEFINED _MUJOCO_RENDER_TEST_MOCK_OSMESA AND _MUJOCO_RENDER_TEST_MOCK_OSMESA)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
    message(STATUS "OSMesa found. Selected for offscreen RenderCore.")
  else()
    find_package(OpenGL COMPONENTS OpenGL EGL QUIET)
    if(OpenGL_EGL_FOUND)
      set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
      message(STATUS "EGL found. Selected for offscreen RenderCore.")
    else()
      find_package(OSMesa QUIET)
      if(OSMesa_FOUND)
        set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
        message(STATUS "OSMesa found. Selected for offscreen RenderCore.")
      else()
        message(WARNING "Neither EGL nor OSMesa found. Offscreen RenderCore disabled.")
      endif()
    endif()
  endif()
endif()

message(STATUS "configured visible GUI backend: ${_MUJOCO_VISIBLE_RENDER_BACKEND}")
message(STATUS "configured offscreen RenderCore backend: ${_MUJOCO_RESOLVED_OFFSCREEN_BACKEND}")

file(MAKE_DIRECTORY ${GENERATED_HEADERS_DIR}/${PROJECT_NAME})
set(_render_backend_header_value ${_MUJOCO_VISIBLE_RENDER_BACKEND})
set(_offscreen_render_backend_header_value ${_MUJOCO_RESOLVED_OFFSCREEN_BACKEND})
configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/header_templates/render_backend.hpp.in
  ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/render_backend.hpp
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
  ${GENERATED_HEADERS_DIR}
)

# Install header file
# catkin_lint: ignore_once external_file
install(FILES ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/render_backend.hpp
  DESTINATION ${GENERATED_HEADERS_INSTALL_DIR}
)
