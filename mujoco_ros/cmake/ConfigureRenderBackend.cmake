include_guard()

find_library(GLFW libglfw.so.3) # Find GLFW3 for GUI

set(RENDER_BACKEND "ANY" CACHE STRING "Choose rendering backend")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "ANY" "GLFW" "EGL" "OSMESA" "DISABLE")

set(NO_GLFW OFF)
set(NO_EGL OFF)
set(NO_OSMESA OFF)
set(NO_RENDER OFF)

if (RENDER_BACKEND STREQUAL "GLFW")
  set(NO_EGL ON)
  set(NO_OSMESA ON)
  message(WARNING "EGL and OSMesa disabled!")
elseif (RENDER_BACKEND STREQUAL "EGL")
  set(NO_GLFW ON)
  message(WARNING "GLFW disabled! Will use OSMesa as fallback if EGL can not be found.")
elseif (RENDER_BACKEND STREQUAL "OSMESA")
  set(NO_GLFW ON)
  set(NO_EGL ON)
  message(WARNING "GLFW and EGL disabled!")
elseif (RENDER_BACKEND STREQUAL "DISABLE")
  set(NO_GLFW ON)
  set(NO_EGL ON)
  set(NO_OSMESA ON)
  set(RENDERING_BACKEND "NO")
  set(NO_RENDER ON)
  message(WARNING "GLFW, EGL and OSMesa disabled! No rendering will be available.")
endif()
# ELSE: ANY

if (NOT NO_RENDER)
  if (NO_GLFW OR ${GLFW} STREQUAL "GLFW-NOTFOUND")
    if (NOT NO_GLFW)
      message(WARNING "GLFW3 not found. GUI will not be available.")
    endif()

    find_package(OpenGL COMPONENTS OpenGL EGL) # Find OpenGL (EGL) for offscreen rendering
    if (NO_EGL OR ${OpenGL_EGL_FOUND} STREQUAL "FALSE")
      if (NOT NO_EGL)
        message(WARNING "EGL not found. Falling back to OSMesa.")
      endif()

      find_package(OSMesa)

      if (NO_OSMESA OR !OSMesa_FOUND)
        if(NOT NO_OSMESA)
          message(WARNING "OSMesa not found.")
        endif()
        set(RENDERING_BACKEND "NO")
      else() # OSMesa found
        set(RENDERING_BACKEND "OSMESA")
        message(STATUS "OSMesa found. Offscreen rendering available.")
      endif()
    else() # EGL found
      set(RENDERING_BACKEND "EGL")
      message(STATUS "EGL found. Offscreen rendering available.")
    endif()

  else() # GLFW found
    set(RENDERING_BACKEND "GLFW")
    message(STATUS "GLFW3 found. GUI and offscreen rendering available.")
  endif()
endif()

message(STATUS "configured RENDERING_BACKEND: ${RENDERING_BACKEND}")

add_custom_command(
  OUTPUT ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/render_backend.hpp always_rebuild_backend
  COMMAND ${CMAKE_COMMAND}
  -DRENDER_BACKEND=${RENDERING_BACKEND}
  -DGENERATED_HEADERS_DIR=${GENERATED_HEADERS_DIR}/${PROJECT_NAME}
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBackendHeader.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(render_backend_h
 DEPENDS always_rebuild_backend
 COMMENT "Generating render backend header"
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
  ${GENERATED_HEADERS_DIR}
)

# Install header file
# catkin_lint: ignore_once external_file
install(FILES ${GENERATED_HEADERS_DIR}/${PROJECT_NAME}/render_backend.hpp
  DESTINATION ${GENERATED_HEADERS_INSTALL_DIR}
)
