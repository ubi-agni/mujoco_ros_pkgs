include_guard()

option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)

function (_get_gcc_clang_common_warnings WARNING)
  set(${WARNING}
      -Wall
      -Wextra # reasonable and standard
      # -Wshadow # warn the user if a variable declaration shadows one from a parent context
      -Wnon-virtual-dtor # warn the user if a class with virtual functions has a non-virtual destructor. This helps
                         # catch hard to track down memory errors
      -Wold-style-cast # warn for c-style casts
      -Wcast-align # warn for potential performance problem casts
      -Wunused # warn on anything being unused
      -Woverloaded-virtual # warn if you overload (not override) a virtual function
      -Wpedantic # warn if non-standard C++ is used
      # -Wdouble-promotion # warn if float is implicit promoted to double
      -Wformat=2 # warn on security issues around functions that format output (ie printf)
      -Wno-missing-field-initializers # don't warn about shorter initializer lists than there are fields in structs
      -Wno-int-in-bool-context # don't warn if int used in boolean context, eg if (x) where x is int
      -Wno-sign-compare # don't warn if comparing signed and unsigned values
      -Wno-unknown-pragmas # don't warn if unknown pragmas are used
      $<$<BOOL:${WARNINGS_AS_ERRORS}>:-Werror>
      PARENT_SCOPE)
endfunction ()

function (_get_clang_warnings WARNING)
  _get_gcc_clang_common_warnings(COMMON_WARNING)
  set(${WARNING}
      ${COMMON_WARNING}
      -Wno-unknown-warning-option # clang-tidy may inspect compile databases containing GCC-only warning flags
      PARENT_SCOPE)
endfunction ()

function (_get_gcc_warnings WARNING)
  _get_gcc_clang_common_warnings(COMMON_WARNING)
  set(${WARNING}
      ${COMMON_WARNING}
      $<$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,6.1>:-Wmisleading-indentation> # warn if indentation implies blocks where blocks do not exist
      $<$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,6.1>:-Wnull-dereference>
      $<$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,7.1>:-Wimplicit-fallthrough> # warn on statements that fallthrough without an explicit annotation
      # see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=83591
      $<$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,8.0>:-Wduplicated-cond> # warn if if / else chain has duplicated conditions
      $<$<VERSION_GREATER:$<CXX_COMPILER_VERSION>,8.0>:-Wduplicated-branches> # warn if if / else branches have duplicated code
      -Wlogical-op # warn about logical operations being used where bitwise were probably wanted
      -Wuseless-cast # warn if you perform a cast to the same type
      PARENT_SCOPE)
endfunction ()

function (configure_project_warnings)
  cmake_parse_arguments("" "" "TARGET" "WARNINGS" ${ARGN})

  if (NOT _TARGET)
    message(FATAL_ERROR "No target specified")
  endif ()

  if (CMAKE_CXX_COMPILER_ID MATCHES ".*Clang")
    _get_clang_warnings(CXX_PROJECT_WARNING)
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    _get_gcc_warnings(CXX_PROJECT_WARNING)
  else ()
    message(AUTHOR_WARNING "No compiler warnings set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
  endif ()

  target_compile_options(${_TARGET} INTERFACE "$<IF:$<BOOL:${_WARNINGS}>,${_WARNINGS},${CXX_PROJECT_WARNING}>")
endfunction ()
