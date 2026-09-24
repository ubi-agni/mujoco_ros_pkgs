include_guard()

option(ENABLE_IPO "Enable Interprocedural Optimization (IPO), a.k.a Link Time Optimizaition (LTO)" OFF)

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

cmake_policy(POP)
