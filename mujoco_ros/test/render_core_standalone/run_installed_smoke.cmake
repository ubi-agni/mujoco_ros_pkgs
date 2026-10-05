if(NOT DEFINED STANDALONE_SOURCE_DIR OR NOT DEFINED STANDALONE_BINARY_DIR OR NOT DEFINED INSTALL_PREFIX)
  message(FATAL_ERROR "standalone smoke paths are required")
endif()

set(_standalone_coverage_arg)
if(STANDALONE_COVERAGE)
  set(_standalone_coverage_arg -DMJR_STANDALONE_COVERAGE=ON)
endif()

file(REMOVE_RECURSE "${STANDALONE_BINARY_DIR}")
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env --unset=ROS_VERSION
          "${CMAKE_COMMAND}"
          -S "${STANDALONE_SOURCE_DIR}"
          -B "${STANDALONE_BINARY_DIR}"
          "-DCMAKE_PREFIX_PATH=${INSTALL_PREFIX}"
          ${_standalone_coverage_arg}
  RESULT_VARIABLE configure_result
)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "installed RenderCore standalone configure failed: ${configure_result}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${STANDALONE_BINARY_DIR}" --parallel 2
  RESULT_VARIABLE build_result
)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "installed RenderCore standalone build failed: ${build_result}")
endif()

execute_process(
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${STANDALONE_BINARY_DIR}" --output-on-failure
  RESULT_VARIABLE test_result
)
if(NOT test_result EQUAL 0)
  message(FATAL_ERROR "installed RenderCore disabled-backend smoke failed: ${test_result}")
endif()
