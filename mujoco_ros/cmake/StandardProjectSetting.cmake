include_guard()

include(Utility)

function (config_debug_output)
  # we use directory property since we normally want ALL our diagnostic errors of the targets to be colored
  add_compile_options($<$<AND:$<CXX_COMPILER_ID:GNU>,${MATCH_CLANG_COMPILER_ID_GENEX}>:-fdiagnostics-color=always>)
  add_compile_options($<$<AND:$<CXX_COMPILER_ID:MSVC>,$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,19.00>>:/diagnostics:column>)
endfunction ()

function (configure_project_setting)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    # set(CMAKE_C_FLAGS_COVERAGE "-g -O0" CACHE STRING "")
    # set(CMAKE_CXX_FLAGS_COVERAGE "-g -O0" CACHE STRING "")
  elseif (MSVC)
    # MSVC CXX_COMPILER_VERSION is different from MSVC_VERSION
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 19.14)
      set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "/external:I " CACHE STRING "" FORCE)
    endif ()
  endif ()

  get_property(BUILDING_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if (BUILDING_MULTI_CONFIG)
    # Make sure that all supported configuration types have their
    # associated conan packages available. You can reduce this
    # list to only the configuration types you use, but only if one
    # is not forced-set on the command line for VS
    if (DEFINED CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE STREQUAL "")
      message(FATAL_ERROR "CMAKE_BUILD_TYPE='${CMAKE_BUILD_TYPE}' is defined and non-empty "
                          "(but should not be for a multi-configuration generator)")
    endif ()
  else ()
    # Since the recommended optimization flags differ between sanitizers, e.g. ASAN is recommended to build with -O1,
    # but UBSAN should be built with -O0, otherwise undefined behaviors can be optimized away. I don't think we should
    # set a build type for sanitizing
    set(AVAILABLE_BUILD_TYPE "Debug" "Release" "MinSizeRel" "RelWithDebInfo")

    # Set the possible values of build type for cmake-gui, ccmake
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${AVAILABLE_BUILD_TYPE})

    # only determine CMAKE_BUILD_TYPE for single configuration generator
    if (NOT CMAKE_BUILD_TYPE)
      message(FATAL_ERROR "Need to specify a build type as one of the following: ${AVAILABLE_BUILD_TYPE}")
    elseif (NOT CMAKE_BUILD_TYPE IN_LIST AVAILABLE_BUILD_TYPE)
      message(FATAL_ERROR "Unknown build type: ${CMAKE_BUILD_TYPE}")
    endif ()
  endif ()

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Generate compile_commands.json to make it easier to work with clang based tools" FORCE)

  config_debug_output()
endfunction ()
