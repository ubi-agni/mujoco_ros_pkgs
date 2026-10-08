include_guard()

# Reject removed public cache variables (no compatibility aliases).
if(DEFINED CACHE{RENDER_BACKEND})
  message(FATAL_ERROR
    "RENDER_BACKEND was removed. Configure visible GUI with WITH_GUI=ON or WITH_GUI=OFF.")
endif()
if(DEFINED CACHE{OFFSCREEN_RENDER_BACKEND})
  message(FATAL_ERROR
    "OFFSCREEN_RENDER_BACKEND was removed. Configure offscreen RenderCore with "
    "OFFSCREEN_BACKEND=ANY, GLFW, EGL, OSMESA, or DISABLE.")
endif()

set(WITH_GUI "ON" CACHE STRING "Build visible GLFW GUI")
set_property(CACHE WITH_GUI PROPERTY STRINGS "ON" "OFF")
set(OFFSCREEN_BACKEND "ANY" CACHE STRING "Choose offscreen RenderCore backend")
set_property(CACHE OFFSCREEN_BACKEND PROPERTY STRINGS "ANY" "GLFW" "EGL" "OSMESA" "DISABLE")

if(NOT WITH_GUI STREQUAL "ON" AND NOT WITH_GUI STREQUAL "OFF")
  message(FATAL_ERROR "Unknown WITH_GUI='${WITH_GUI}'. Choose ON or OFF.")
endif()

if(NOT OFFSCREEN_BACKEND STREQUAL "ANY" AND
    NOT OFFSCREEN_BACKEND STREQUAL "GLFW" AND
    NOT OFFSCREEN_BACKEND STREQUAL "EGL" AND
    NOT OFFSCREEN_BACKEND STREQUAL "OSMESA" AND
    NOT OFFSCREEN_BACKEND STREQUAL "DISABLE")
  message(FATAL_ERROR
    "Unknown OFFSCREEN_BACKEND='${OFFSCREEN_BACKEND}'. "
    "Choose ANY, GLFW, EGL, OSMESA, or DISABLE.")
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
    # Viewer mjr_makeContext/gladLoadGL must bind libGL.so, not OSMesa's gl*
    # exports. --no-as-needed keeps libGL in DT_NEEDED even without a direct
    # symbol ref from this target. Isolated cmake mock cases skip this.
    if(NOT DEFINED _MUJOCO_RENDER_TEST_MOCK_GLFW)
      find_library(MJR_LIBGL NAMES GL libGL.so.1 REQUIRED)
    endif()
  else()
    message(FATAL_ERROR
      "WITH_GUI=ON requires GLFW3 (libglfw.so.3). Install GLFW or configure with WITH_GUI=OFF.")
  endif()
endif()

set(_offscreen_request "${OFFSCREEN_BACKEND}")

set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "NO")
if(_offscreen_request STREQUAL "DISABLE")
  message(STATUS "Offscreen RenderCore disabled.")
elseif(_offscreen_request STREQUAL "GLFW")
  if(NOT WITH_GUI STREQUAL "ON")
    message(FATAL_ERROR
      "OFFSCREEN_BACKEND=GLFW requires WITH_GUI=ON (hidden GLFW offscreen reuses the viewer GL stack). "
      "Use OFFSCREEN_BACKEND=EGL or OSMESA with WITH_GUI=OFF, or OFFSCREEN_BACKEND=ANY.")
  endif()
  set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "GLFW")
  message(STATUS "Hidden GLFW selected for offscreen RenderCore (OFFSCREEN_BACKEND=GLFW).")
elseif(_offscreen_request STREQUAL "EGL")
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_EGL AND _MUJOCO_RENDER_TEST_MOCK_EGL)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "EGL selected for offscreen RenderCore.")
  else()
    find_package(OpenGL COMPONENTS OpenGL EGL REQUIRED)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "EGL selected for offscreen RenderCore.")
  endif()
elseif(_offscreen_request STREQUAL "OSMESA" AND WITH_GUI STREQUAL "ON")
  # OSMesa exports gl* symbols that collide with the viewer's libGL in one process.
  message(FATAL_ERROR
    "WITH_GUI=ON cannot be combined with OFFSCREEN_BACKEND=OSMESA (OSMesa and the GLFW viewer's "
    "libGL conflict in-process). Use OFFSCREEN_BACKEND=EGL (GPU, or llvmpipe if software EGL is "
    "acceptable), or OFFSCREEN_BACKEND=GLFW (or ANY) to reuse GLFW for offscreen rendering. "
    "Use WITH_GUI=OFF for OSMesa-only headless builds.")
elseif(_offscreen_request STREQUAL "OSMESA")
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_OSMESA AND _MUJOCO_RENDER_TEST_MOCK_OSMESA)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
    message(STATUS "OSMesa selected for offscreen RenderCore.")
  else()
    find_package(OSMesa REQUIRED)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "OSMESA")
    message(STATUS "OSMesa selected for offscreen RenderCore.")
  endif()
elseif(WITH_GUI STREQUAL "ON")
  # A hidden GLFW context on the RenderCore thread reuses the viewer's GL stack; libEGL merely
  # being installed says nothing about a usable GPU, so EGL is chosen only on proven hardware.
  set(_use_hw_egl FALSE)
  set(_has_dri_render_nodes FALSE)
  set(_egl_glfw_fallback "")
  if(DEFINED _MUJOCO_RENDER_TEST_MOCK_EGL_DEVICE)
    if(_MUJOCO_RENDER_TEST_MOCK_EGL_DEVICE)
      set(_use_hw_egl TRUE)
      set(_has_dri_render_nodes TRUE)
    endif()
  else()
    file(GLOB _dri_render_nodes "/dev/dri/renderD*")
    if(_dri_render_nodes)
      set(_has_dri_render_nodes TRUE)
      find_package(OpenGL COMPONENTS OpenGL EGL QUIET)
      if(OpenGL_EGL_FOUND)
        try_run(_egl_probe_run _egl_probe_compile
          ${CMAKE_BINARY_DIR}/egl_hw_probe
          ${CMAKE_CURRENT_LIST_DIR}/egl_hw_probe.c
          LINK_LIBRARIES OpenGL::EGL OpenGL::GL
          RUN_OUTPUT_VARIABLE _egl_probe_stdout)
        string(STRIP "${_egl_probe_stdout}" _egl_probe_stdout)
        if(NOT _egl_probe_compile)
          set(_egl_glfw_fallback "compile failed")
        elseif(_egl_probe_run EQUAL 0)
          set(_use_hw_egl TRUE)
          message(STATUS
            "EGL hardware probe (eglGetDisplay default): hardware accepted. "
            "${_egl_probe_stdout}")
        elseif(_egl_probe_run EQUAL 7)
          set(_egl_glfw_fallback "software renderer only; ${_egl_probe_stdout}")
        else()
          set(_egl_glfw_fallback "run failed (exit ${_egl_probe_run})")
        endif()
      else()
        set(_egl_glfw_fallback "DRI render nodes present but EGL not found; hardware probe skipped")
      endif()
    endif()
  endif()
  if(_egl_glfw_fallback)
    message(WARNING
      "EGL hardware probe (eglGetDisplay default): ${_egl_glfw_fallback}; "
      "offscreen ANY will use hidden GLFW.")
  endif()
  if(_use_hw_egl)
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "EGL")
    message(STATUS "Hardware EGL device found. Selected for offscreen RenderCore.")
  else()
    set(_MUJOCO_RESOLVED_OFFSCREEN_BACKEND "GLFW")
    if(NOT _has_dri_render_nodes)
      message(STATUS
        "No DRI render node (/dev/dri/renderD*); using hidden GLFW context for offscreen RenderCore.")
    else()
      message(STATUS "Using hidden GLFW context for offscreen RenderCore.")
    endif()
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
