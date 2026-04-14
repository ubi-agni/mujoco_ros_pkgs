if(NOT DEFINED MUJOCO_DIR AND DEFINED ENV{MUJOCO_DIR})
    set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
endif()

if(NOT DEFINED MUJOCO_DIR OR MUJOCO_DIR STREQUAL "")
    message(FATAL_ERROR "MUJOCO_DIR is not set")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
    message(FATAL_ERROR "Could not find ${MUJOCO_DIR}/include/mujoco/mujoco.h")
endif()

file(GLOB MUJOCO_SHARED_LIBS
    "${MUJOCO_DIR}/lib/libmujoco.so*"
)

if(NOT MUJOCO_SHARED_LIBS)
    message(FATAL_ERROR "Could not find libmujoco.so* in ${MUJOCO_DIR}/lib")
endif()

install(
    DIRECTORY "${MUJOCO_DIR}/include/"
    DESTINATION include
)

install(
    FILES ${MUJOCO_SHARED_LIBS}
    DESTINATION lib
)

if(EXISTS "${MUJOCO_DIR}/plugin")
    install(
        DIRECTORY "${MUJOCO_DIR}/plugin/"
        DESTINATION lib/mujoco_plugin
    )
endif()
