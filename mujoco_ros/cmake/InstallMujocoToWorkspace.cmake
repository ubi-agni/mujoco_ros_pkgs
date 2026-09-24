if(NOT DEFINED MUJOCO_DIR AND DEFINED ENV{MUJOCO_DIR})
    set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
endif()

if(NOT DEFINED MUJOCO_DIR OR MUJOCO_DIR STREQUAL "")
    message(FATAL_ERROR "MUJOCO_DIR is not set")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "Could not find ${MUJOCO_DIR}/include/mujoco/mujoco.h")
endif()

set(MUJOCO_REAL_LIB "${MUJOCO_DIR}/lib/libmujoco.so.3.3.5")
if(NOT EXISTS "${MUJOCO_REAL_LIB}" OR IS_SYMLINK "${MUJOCO_REAL_LIB}")
    message(FATAL_ERROR "Could not find the real MuJoCo 3.3.5 library at ${MUJOCO_REAL_LIB}")
endif()

# ament symlink-install rewrites install(FILES|DIRECTORY) to the package's
# original install prefix, ignoring `cmake --install --prefix`. The
# render_core_standalone_installed smoke test stages into a fresh prefix, so
# MuJoCo must be placed via install(CODE) which honors CMAKE_INSTALL_PREFIX.
install(CODE
    "
    set(_mujoco_include \"${MUJOCO_DIR}/include\")
    set(_mujoco_real_lib \"${MUJOCO_REAL_LIB}\")
    set(_mujoco_plugin \"${MUJOCO_DIR}/plugin\")
    file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/include\")
    file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/lib\")
    file(COPY \"\${_mujoco_include}/\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/include\")
    file(COPY \"\${_mujoco_real_lib}\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib\")
    file(REMOVE \"\${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so\")
    file(CREATE_LINK \"libmujoco.so.3.3.5\" \"\${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so\" SYMBOLIC)
    if(EXISTS \"\${_mujoco_plugin}\")
      file(MAKE_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}/lib/mujoco_plugin\")
      file(COPY \"\${_mujoco_plugin}/\" DESTINATION \"\${CMAKE_INSTALL_PREFIX}/lib/mujoco_plugin\")
    endif()
    "
)
