find_package(mujoco QUIET NO_MODULE)

if (mujoco_FOUND AND mujoco_FIND_VERSION)
	if (NOT mujoco_FIND_VERSION_MAJOR EQUAL mujoco_VERSION_MAJOR OR mujoco_FIND_VERSION_MINOR GREATER mujoco_VERSION_MINOR OR mujoco_FIND_VERSION_PATCH GREATER mujoco_VERSION_PATCH)
		message(STATUS "Requested MuJoCo version ${mujoco_FIND_VERSION} but found incompatible version ${mujoco_VERSION}")
		unset(mujoco_FOUND)
	endif()
	message(STATUS "Found MuJoCo version ${mujoco_VERSION} at ${mujoco_DIR}")
endif()

function(_get_imported_location tgt out_var)
  foreach(cfg "" "_DEBUG" "_RELEASE" "_RELWITHDEBINFO" "_MINSIZEREL")
    get_target_property(loc "${tgt}" IMPORTED_LOCATION${cfg})
    if(loc)
      set(${out_var} "${loc}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  message(FATAL_ERROR
    "Could not find any IMPORTED_LOCATION* property on target ${tgt}")
endfunction()

if (mujoco_FOUND)
	_get_imported_location(mujoco::mujoco mujoco_LIBRARIES)

	get_filename_component(_MJ_LIB_DIR
		"${mujoco_LIBRARIES}"
		DIRECTORY
	)
	get_filename_component(MUJOCO_ROOT_DIR
		"${_MJ_LIB_DIR}"
		DIRECTORY
	)

	set(MUJOCO_DIR "${MUJOCO_ROOT_DIR}" CACHE PATH "Path to MuJoCo installation directory")
else()
	message(STATUS "Looking for MuJoCo tar install ...")
	# Initialize MUJOCO_DIR from environment variable if not yet set as cmake variable
	if(NOT DEFINED MUJOCO_DIR)
		set(MUJOCO_DIR $ENV{MUJOCO_DIR} CACHE PATH "Path to MuJoCo installation directory")
	endif()
	# Find headers
	find_file(mujoco_INCLUDE_DIRS include/mujoco/mujoco.h PATHS ${MUJOCO_DIR})
	if(mujoco_INCLUDE_DIRS)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
	endif()

	# Find library
	find_library(mujoco_LIBRARIES lib/libmujoco.so PATHS ${MUJOCO_DIR})

	# Find dependencies
	cmake_policy(SET CMP0072 NEW)
	include(CMakeFindDependencyMacro)
	find_package(OpenGL)

	if(mujoco_INCLUDE_DIRS AND mujoco_LIBRARIES)
		set(mujoco_FOUND TRUE)
		add_library(mujoco::mujoco SHARED IMPORTED)
		set_property(TARGET mujoco::mujoco PROPERTY IMPORTED_LOCATION "${mujoco_LIBRARIES}")
		set_target_properties(mujoco::mujoco PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIRS}"
		)
	else()
		message(FATAL_ERROR "Failed to find mujoco (MUJOCO_DIR=${MUJOCO_DIR})")
	endif()

	if(mujoco_FIND_VERSION)
		message(NOTICE "MuJoCo tar install found, but version can not be detected. If experiencing errors check that version ${mujoco_FIND_VERSION} or a newer, compatible version is installed")
	endif()
endif()

set(mujoco_LIBRARIES mujoco::mujoco)
get_target_property(mujoco_INCLUDE_DIRS mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
