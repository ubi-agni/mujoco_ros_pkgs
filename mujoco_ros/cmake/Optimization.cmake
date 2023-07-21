include_guard()
include(CheckAVXSupport)

option(ENABLE_IPO "Enable Interprocedural Optimization (IPO), a.k.a Link Time Optimizaition (LTO)" OFF)
option(ENABLE_AVX "Build binaries that require AVX instructions, if possible" ON)
option(ENABLE_AVX_INTRINSICS "Make use of hand-written AVX intrinsics, if possible" ON)

cmake_policy(PUSH)

if (POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif ()

function (configure_interprocedural_optimization)
  cmake_parse_arguments("" "" "" "DISABLE_FOR_CONFIG" ${ARGN})

  foreach (config_type IN LISTS _DISABLE_FOR_CONFIG)
    string(TOUPPER ${config_type} config_type)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_${config_type} OFF CACHE INTERNAL "IPO is disabled for ${config_type}" FORCE)
  endforeach ()

  if (NOT ${ENABLE_IPO})
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF CACHE INTERNAL "" FORCE)
    return()
  endif ()

  include(CheckIPOSupported)
  check_ipo_supported(RESULT result OUTPUT output LANGUAGES CXX)
  if (result)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON CACHE INTERNAL "" FORCE)
  else ()
    message(WARNING "Interprocedural Optimization is not supported. Reason: ${output}")
  endif ()
endfunction ()

function(configure_project_avx_support)
  cmake_parse_arguments("" "" "TARGET" "AVX" ${ARGN})

  if(NOT _TARGET)
    message(FATAL_ERROR "No target specified")
  endif()

  if(ENABLE_AVX)
    get_avx_compile_options(AVX_COMPILE_OPTIONS)
  else()
    message(STATUS "AVX support disabled manually")
    set(AVX_COMPILE_OPTIONS "")
  endif()

  target_compile_options(${_TARGET} INTERFACE "$<IF:$<BOOL:${_AVX}>,${_AVX},${AVX_COMPILE_OPTIONS}>")
endfunction()

cmake_policy(POP)
