# Generate header file with rendering backend definition
file(MAKE_DIRECTORY ${HEADER_FILE_PATH})
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/render_backend.h.in
  ${HEADER_FILE_PATH}/render_backend.h
)
