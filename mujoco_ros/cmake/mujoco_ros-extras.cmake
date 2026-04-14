get_filename_component(_MUJOCO_ROS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

find_library(_MUJOCO_ROS_MUJOCO_LIB
  NAMES mujoco libmujoco.so
  PATHS "${_MUJOCO_ROS_PREFIX}/lib"
  NO_DEFAULT_PATH
)

if(NOT _MUJOCO_ROS_MUJOCO_LIB)
  message(FATAL_ERROR "mujoco_ros: installed libmujoco not found under ${_MUJOCO_ROS_PREFIX}/lib")
endif()

if(NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${_MUJOCO_ROS_MUJOCO_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${_MUJOCO_ROS_PREFIX}/include"
  )
endif()

set(mujoco_FOUND TRUE)
set(mujoco_INCLUDE_DIRS "${_MUJOCO_ROS_PREFIX}/include")
set(mujoco_LIBRARIES mujoco::mujoco)
