if(NOT DEFINED EXPORT_FILE OR "${EXPORT_FILE}" STREQUAL "")
  message(FATAL_ERROR "EXPORT_FILE must be defined and non-empty")
endif()

if(NOT DEFINED EXPECTED_MUJOCO_PREFIX OR "${EXPECTED_MUJOCO_PREFIX}" STREQUAL "")
  message(FATAL_ERROR "EXPECTED_MUJOCO_PREFIX must be defined and non-empty")
endif()

if(NOT EXISTS "${EXPORT_FILE}")
  message(FATAL_ERROR "Installed export does not exist: ${EXPORT_FILE}")
endif()

file(READ "${EXPORT_FILE}" _export)
string(FIND "${_export}" "get_filename_component(_MUJOCO_ROS_PREFIX \"\${CMAKE_CURRENT_LIST_DIR}/../../..\" ABSOLUTE)" _prefix_derivation)
if(_prefix_derivation LESS 0)
  message(FATAL_ERROR "Installed export does not derive MuJoCo from the installed package prefix")
endif()

string(FIND "${_export}" [=[PATHS "${_MUJOCO_ROS_PREFIX}/lib"]=] _library_path)
if(_library_path LESS 0)
  message(FATAL_ERROR "Installed export does not search the installed package lib directory")
endif()
