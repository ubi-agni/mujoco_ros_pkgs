include_guard()

include(CheckCSourceCompiles)

#
# configure_linker(
#   TARGET target
#   LINKER linker
#   [LINKER_PATH dir]
# )
#
# This function configures linker by checking if the linker is supported by compiler, if yes,
# link the target with -fuse-ld=linker, otherwise it links the target with -B/linker/path if
# compiler support -B option and LINKER_PATH is specified.
#
function(configure_linker)
  cmake_parse_arguments("" "" "TARGET" "" ${ARGN})

  if(NOT _TARGET)
    message(FATAL_ERROR "No target specified!")
  endif()

  set(EXTRA_LINK_OPTIONS)

  set(CMAKE_REQUIRED_FLAGS "-fuse-ld=lld")
  check_c_source_compiles("int main() {}" SUPPORTS_LLD)
  if(SUPPORTS_LLD)
    set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -fuse-ld=lld)
  else()
    set(CMAKE_REQUIRED_FLAGS "-fuse-ld=gold")
    check_c_source_compiles("int main() {}" SUPPORTS_GOLD)
    if(SUPPORTS_GOLD)
      set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -fuse-ld=gold)
    endif()
  endif()

  set(CMAKE_REQUIRED_FLAGS ${EXTRA_LINK_OPTIONS} "-Wl,--gc-sections")
  check_c_source_compiles("int main() {}" SUPPORTS_GC_SECTIONS)
  if(SUPPORTS_GC_SECTIONS)
    set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -Wl,--gc-sections)
  else()
    set(CMAKE_REQUIRED_FLAGS ${EXTRA_LINK_OPTIONS} "-Wl,-dead_strip")
    check_c_source_compiles("int main() {}" SUPPORTS_DEAD_STRIP)
    if(SUPPORTS_DEAD_STRIP)
      set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} -Wl,-dead_strip)
    endif()
  endif()

  target_link_options(${_TARGET} INTERFACE ${EXTRA_LINK_OPTIONS})
endfunction ()
