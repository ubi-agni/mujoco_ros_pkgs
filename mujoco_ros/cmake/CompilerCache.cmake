include_guard()

function (_configure_ccache)
  cmake_parse_arguments("" "" "BASE_DIR" "" ${ARGN})

  find_program(CCACHE_BINARY ccache)
  if (NOT _BASE_DIR)
    if (PROJECT_IS_TOP_LEVEL)
      execute_process(COMMAND catkin locate OUTPUT_VARIABLE _BASE_DIR ERROR_VARIABLE CATKIN_ERR RESULTS_VARIABLE TERM_RESULT)
      string(REGEX REPLACE "\n" "" _BASE_DIR "${_BASE_DIR}")
    else ()
      set(_BASE_DIR ${CMAKE_SOURCE_DIR}) # catkin_make tool use add_subdirectory() to invoke ros pkg cmakelist
    endif ()
  endif ()

  if (CATKIN_ERR OR (TERM_RESULT AND NOT "${TERM_RESULT}" STREQUAL "0"))
    message(STATUS "Can't identify catkin workspace root directory, neither did user provide one, using default config")
  else ()
    set(CCACHE_BASE_DIR_ARG CCACHE_BASEDIR=${_BASE_DIR})
  endif ()

  # see https://ccache.dev/manual/4.6.1.html base_dir
  if (CCACHE_BINARY)
    message(STATUS "ccache found and enabled")
    set(ccacheEnv
        CCACHE_CPP2=true ${CCACHE_BASE_DIR_ARG}
        # This comes with a theoretical risk of a race condition, but for typical scenarios,
        # that race condition is highly unlikely
        CCACHE_SLOPINESS=include_file_ctime,include_file_mtime,time_macros)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_BINARY} CACHE STRING "CXX compiler cache used")
    set(CMAKE_C_COMPILER_LAUNCHER ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_BINARY} CACHE STRING "C compiler cache used")
  else ()
    message(WARNING "ccache is enabled but was not found. Not using it")
  endif ()
endfunction ()

function (configure_compiler_cache)
  cmake_parse_arguments("CACHE" "" "OPTION" "" ${ARGN})
  cmake_parse_arguments(PARSE_ARGV 1 FWD "" "" "")

  set(CACHE_OPTION_VALUES "ccache")
  list(FIND CACHE_OPTION_VALUES ${CACHE_OPTION} CACHE_OPTION_INDEX)
  if (NOT ${CACHE_OPTION_INDEX} EQUAL -1)
    cmake_language(CALL _configure_${CACHE_OPTION} ${FWD_UNPARSED_ARGUMENTS})
  else ()
    message(STATUS "Using compiler cache: '${CACHE_OPTION}', explicitly supported entries are ${CACHE_OPTION_VALUES}")

    find_program(CACHE_BINARY ${CACHE_OPTION})
    if (CACHE_BINARY)
      message(STATUS "${CACHE_OPTION} found and enabled")
      set(CMAKE_CXX_COMPILER_LAUNCHER ${CACHE_BINARY} CACHE STRING "CXX compiler cache used")
      set(CMAKE_C_COMPILER_LAUNCHER ${CACHE_BINARY} CACHE STRING "C compiler cache used")
    else ()
      message(WARNING "${CACHE_OPTION} is enabled but was not found. Not using it")
    endif ()
  endif ()
endfunction ()
