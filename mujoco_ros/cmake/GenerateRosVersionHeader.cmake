file(MAKE_DIRECTORY ${GENERATED_HEADERS_DIR})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/header_templates/ros_version.hpp.in
  ${GENERATED_HEADERS_DIR}/ros_version.hpp
)
