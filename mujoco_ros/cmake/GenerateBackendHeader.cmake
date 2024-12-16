# Generate header file with rendering backend definition
file(MAKE_DIRECTORY ${GENERATED_HEADERS_DIR})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/header_templates/render_backend.hpp.in
  ${GENERATED_HEADERS_DIR}/render_backend.hpp
)
