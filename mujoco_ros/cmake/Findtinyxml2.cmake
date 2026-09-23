# Find tinyxml2 from either a CMake package config or a plain system install.
#
# Provides:
#   tinyxml2::tinyxml2
#   tinyxml2_FOUND

find_path(tinyxml2_INCLUDE_DIR
  NAMES tinyxml2.h
)

find_library(tinyxml2_LIBRARY
  NAMES tinyxml2
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(tinyxml2
  REQUIRED_VARS tinyxml2_INCLUDE_DIR tinyxml2_LIBRARY
)

if(tinyxml2_FOUND AND NOT TARGET tinyxml2::tinyxml2)
  add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
  set_target_properties(tinyxml2::tinyxml2 PROPERTIES
    IMPORTED_LOCATION "${tinyxml2_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${tinyxml2_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(tinyxml2_INCLUDE_DIR tinyxml2_LIBRARY)
