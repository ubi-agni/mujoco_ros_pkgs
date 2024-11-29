include_guard()

find_library(GLFW libglfw.so.3) # Find GLFW3 for GUI

set(RENDER_BACKEND "ANY" CACHE STRING "Choose rendering backend")
set_property(CACHE RENDER_BACKEND PROPERTY STRINGS "ANY" "USE_GLFW" "USE_EGL" "USE_OSMESA")
message(STATUS "configured RENDER_BACKEND: ${RENDER_BACKEND}")

if (RENDER_BACKEND STREQUAL "ANY")
  unset(NO_GLFW)
  unset(NO_EGL)
  unset(NO_OSMESA)
endif()

if (RENDER_BACKEND STREQUAL "USE_GLFW")
  set(NO_EGL ON)
  set(NO_OSMESA ON)
  message(STATUS "EGL and OSMESA disabled!")
endif()

if (RENDER_BACKEND STREQUAL "USE_EGL")
  set(NO_GLFW ON)
  message(STATUS "GLFW disabled! Will use OSMesa as fallback if EGL can not be found.")
endif()

if (RENDER_BACKEND STREQUAL "USE_OSMESA")
  set(NO_GLFW ON)
  set(NO_EGL ON)
  message(STATUS "GLFW and EGL disabled!")
endif()

if (NO_GLFW OR ${GLFW} STREQUAL "GLFW-NOTFOUND")
  message(WARNING "GLFW3 not found or disabled. GUI will not be available.")

  find_package(OpenGL COMPONENTS OpenGL EGL) # Find OpenGL (EGL) for offscreen rendering
  if (NO_EGL OR ${OpenGL_EGL_FOUND} STREQUAL "FALSE")
    message(WARNING "EGL not found or disabled. Falling back to OSMESA.")

    find_package(OSMesa)

    if (NO_OSMESA OR !OSMesa_FOUND)
      message(WARNING "EGL disabled or not found and OSMesa could not be found. Offscreen rendering will not be available!")
      set(RENDERING_BACKEND "USE_NONE")
    else() # OSMesa found
      set(RENDERING_BACKEND "USE_OSMESA")
      message(STATUS "OSMesa found. Offscreen rendering available.")
    endif()

  else() # EGL found
    set(RENDERING_BACKEND "USE_EGL")
    message(STATUS "EGL found. Offscreen rendering available.")
  endif()

else() # GLFW found
  set(RENDERING_BACKEND "USE_GLFW")
  message(STATUS "GLFW3 found. GUI and offscreen rendering available.")
endif()

add_custom_command(
  OUTPUT ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/render_backend.h always_rebuild
  COMMAND ${CMAKE_COMMAND}
  -DRENDER_BACKEND=${RENDERING_BACKEND}
  -DHEADER_FILE_PATH=${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GenerateBackendHeader.cmake
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(render_backend_h
 DEPENDS always_rebuild
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
  ${CATKIN_DEVEL_PREFIX}/include
)

# Install header file
# catkin_lint: ignore_once external_file
install(FILES ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/render_backend.h
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
