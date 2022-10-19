find_package(mujoco QUIET NO_MODULE)

if(NOT mujoco_FOUND)
	# Find headers
	find_file(mujoco_INCLUDE_DIRS include/mujoco/mujoco.h PATHS ENV MUJOCO_DIR)
	if(mujoco_INCLUDE_DIRS)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
		get_filename_component(mujoco_INCLUDE_DIRS ${mujoco_INCLUDE_DIRS} PATH)
	endif()

	# Find library
	find_library(mujoco_LIBRARIES lib/libmujoco.so PATHS ENV MUJOCO_DIR)

	# Find dependencies
	cmake_policy(SET CMP0072 NEW)
	include(CMakeFindDependencyMacro)
	find_dependency(OpenGL REQUIRED)

	if(mujoco_INCLUDE_DIRS AND mujoco_LIBRARIES)
		set(mujoco_FOUND TRUE)
		add_library(mujoco::mujoco SHARED IMPORTED)
		set_property(TARGET mujoco::mujoco PROPERTY IMPORTED_LOCATION "${mujoco_LIBRARIES}")
		set_target_properties(mujoco::mujoco PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIRS}"
		)
	else()
		message(FATAL_ERROR "Failed to find mujoco")
	endif()
endif()

set(mujoco_LIBRARIES mujoco::mujoco)
get_target_property(mujoco_INCLUDE_DIRS mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
