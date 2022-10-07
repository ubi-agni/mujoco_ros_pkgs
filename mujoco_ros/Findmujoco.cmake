find_package(mujoco QUIET NO_MODULE)

if(NOT mujoco_FOUND)
	# Find headers
	find_file(mujoco_INCLUDE_DIR include/mujoco/mujoco.h PATHS ENV MUJOCO_DIR)
	if(mujoco_INCLUDE_DIR)
		get_filename_component(mujoco_INCLUDE_DIR ${mujoco_INCLUDE_DIR} PATH)
		get_filename_component(mujoco_INCLUDE_DIR ${mujoco_INCLUDE_DIR} PATH)
	endif()

	# Find library
	find_library(mujoco_LIBRARIES lib/libmujoco.so PATHS ENV MUJOCO_DIR)

	# Find dependencies
	cmake_policy(SET CMP0072 NEW)
	include(CMakeFindDependencyMacro)
	find_dependency(OpenGL REQUIRED)

	if(mujoco_INCLUDE_DIR AND mujoco_LIBRARIES)
		set(mujoco_FOUND TRUE)
		add_library(mujoco SHARED IMPORTED)
		set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION "${mujoco_LIBRARIES}")
		set_target_properties(mujoco PROPERTIES
			INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIR}"
		)
	else()
		message(FATAL_ERROR "Failed to find mujoco")
	endif()
endif()
