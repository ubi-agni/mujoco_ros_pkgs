set(_mujoco_authoritative FALSE)
if(DEFINED MUJOCO_DIR AND NOT MUJOCO_DIR STREQUAL "")
	# An explicit root must not be bypassed by caller discovery state.
	set(_mujoco_authoritative TRUE)
	unset(mujoco_FOUND)
	unset(mujoco_FOUND CACHE)
	unset(mujoco_VERSION)
	unset(mujoco_VERSION CACHE)
else()
	unset(mujoco_FOUND)
	unset(mujoco_FOUND CACHE)
	find_package(mujoco QUIET NO_MODULE)
endif()

if(NOT _mujoco_authoritative AND mujoco_FOUND AND mujoco_FIND_VERSION)
	if (NOT mujoco_FIND_VERSION_MAJOR EQUAL mujoco_VERSION_MAJOR OR mujoco_FIND_VERSION_MINOR GREATER mujoco_VERSION_MINOR OR mujoco_FIND_VERSION_PATCH GREATER mujoco_VERSION_PATCH)
		message(WARNING "Requested MuJoCo version ${mujoco_FIND_VERSION} but found incompatible version ${mujoco_VERSION}")
		unset(mujoco_FOUND)
	endif()
endif()

if(_mujoco_authoritative OR NOT mujoco_FOUND)
	message(STATUS "Looking for MuJoCo tar install ...")
	# Initialize MUJOCO_DIR from environment variable if not yet set as cmake variable
	if(NOT DEFINED MUJOCO_DIR)
		set(MUJOCO_DIR $ENV{MUJOCO_DIR} CACHE PATH "Path to MuJoCo installation directory")
	endif()
	set(_MUJOCO_SEARCH_OPTIONS)
	if(DEFINED MUJOCO_DIR AND NOT MUJOCO_DIR STREQUAL "")
		set(_MUJOCO_SEARCH_OPTIONS NO_DEFAULT_PATH)
	endif()

	# Avoid reusing cached discovery results from a different MuJoCo root.
	unset(mujoco_INCLUDE_DIRS)
	unset(mujoco_INCLUDE_DIRS CACHE)
	unset(mujoco_LIBRARIES)
	unset(mujoco_LIBRARIES CACHE)

	# Find headers
	find_path(mujoco_INCLUDE_DIRS NAMES mujoco/mujoco.h PATHS "${MUJOCO_DIR}/include"
		${_MUJOCO_SEARCH_OPTIONS}
	)

	# Find library
	find_library(mujoco_LIBRARIES NAMES mujoco libmujoco.so PATHS "${MUJOCO_DIR}/lib"
		${_MUJOCO_SEARCH_OPTIONS}
	)

	# Find dependencies
	cmake_policy(SET CMP0072 NEW)
	include(CMakeFindDependencyMacro)
	find_package(OpenGL)

	if(mujoco_INCLUDE_DIRS AND mujoco_LIBRARIES)
		set(mujoco_FOUND TRUE)
		if(TARGET mujoco::mujoco)
			get_target_property(_mujoco_existing_imported mujoco::mujoco IMPORTED)
			if(NOT _mujoco_existing_imported)
				message(FATAL_ERROR
					"MuJoCo target already exists and cannot be replaced for "
					"MUJOCO_DIR=${MUJOCO_DIR}"
				)
			endif()
			set(_mujoco_location_properties
				IMPORTED_LOCATION
				IMPORTED_LOCATION_NOCONFIG
				IMPORTED_LOCATION_DEBUG
				IMPORTED_LOCATION_RELEASE
				IMPORTED_LOCATION_RELWITHDEBINFO
				IMPORTED_LOCATION_MINSIZEREL
			)
			get_target_property(_mujoco_imported_configurations
				mujoco::mujoco IMPORTED_CONFIGURATIONS
			)
			if(_mujoco_imported_configurations AND
				NOT _mujoco_imported_configurations MATCHES "-NOTFOUND$")
				foreach(_mujoco_imported_configuration IN LISTS
					_mujoco_imported_configurations
				)
					string(TOUPPER "${_mujoco_imported_configuration}"
						_mujoco_imported_configuration_upper
					)
					list(APPEND _mujoco_location_properties
						"IMPORTED_LOCATION_${_mujoco_imported_configuration_upper}"
					)
				endforeach()
			endif()
			list(REMOVE_DUPLICATES _mujoco_location_properties)
			set(_mujoco_existing_location_found FALSE)
			get_filename_component(_mujoco_expected_location
				"${mujoco_LIBRARIES}" REALPATH
			)
			foreach(_mujoco_location_property IN LISTS
				_mujoco_location_properties
			)
				get_target_property(_mujoco_candidate_location
					mujoco::mujoco ${_mujoco_location_property}
				)
				if(_mujoco_candidate_location AND
					NOT _mujoco_candidate_location MATCHES "-NOTFOUND$")
					set(_mujoco_existing_location_found TRUE)
					get_filename_component(_mujoco_actual_location
						"${_mujoco_candidate_location}" REALPATH
					)
					if(NOT _mujoco_actual_location STREQUAL
						_mujoco_expected_location
					)
						message(FATAL_ERROR
							"Existing MuJoCo target configuration "
							"${_mujoco_location_property} does not match "
							"MUJOCO_DIR=${MUJOCO_DIR}: "
							"${_mujoco_candidate_location}"
						)
					endif()
				endif()
			endforeach()
			if(NOT _mujoco_existing_location_found)
				message(FATAL_ERROR
					"Existing MuJoCo target has no imported library location for "
					"MUJOCO_DIR=${MUJOCO_DIR}"
				)
			endif()
			get_target_property(_mujoco_existing_includes
				mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES
			)
			list(FIND _mujoco_existing_includes "${mujoco_INCLUDE_DIRS}"
				_mujoco_include_index
			)
			if(_mujoco_include_index LESS 0)
				message(FATAL_ERROR
					"Existing MuJoCo target includes a different root: "
					"${_mujoco_existing_includes}"
				)
			endif()
		else()
			add_library(mujoco::mujoco SHARED IMPORTED)
			set_property(TARGET mujoco::mujoco PROPERTY
				IMPORTED_LOCATION "${mujoco_LIBRARIES}"
			)
			set_target_properties(mujoco::mujoco PROPERTIES
				INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIRS}"
			)
		endif()
	else()
		message(FATAL_ERROR "Failed to find mujoco (MUJOCO_DIR=${MUJOCO_DIR})")
	endif()

	if(mujoco_FIND_VERSION)
		message(NOTICE "MuJoCo tar install found, but version can not be detected. If experiencing errors check that version ${mujoco_FIND_VERSION} or a newer, compatible version is installed")
	endif()
endif()

set(mujoco_LIBRARIES mujoco::mujoco)
get_target_property(mujoco_INCLUDE_DIRS mujoco::mujoco INTERFACE_INCLUDE_DIRECTORIES)
