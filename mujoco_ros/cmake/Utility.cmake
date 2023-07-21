include_guard()

# generator expression doesn't provide regex match yet (2022/07/14), so we need to manually list all Clang compiler id
set(MATCH_CLANG_COMPILER_ID_GENEX
    $<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:ARMClang>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:FujitsuClang>,$<CXX_COMPILER_ID:ROCMClang>,$<CXX_COMPILER_ID:XLClang>>
)

# https://stackoverflow.com/questions/66823316/set-a-target-vs-global-property-that-applies-to-all-targets-using-cmake
# Get all the CMake targets in the given directory
macro (get_all_targets_recursive targets dir)
  get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
  foreach (subdir ${subdirectories})
    get_all_targets_recursive(${targets} ${subdir})
  endforeach ()

  get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
  list(APPEND ${targets} ${current_targets})
endmacro ()

function (get_all_targets var)
  get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
  set(${var} ${targets} PARENT_SCOPE)
endfunction ()
