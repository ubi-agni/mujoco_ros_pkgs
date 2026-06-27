
if(ROS_VERSION EQUAL 1)
  find_package(rostest REQUIRED)
  find_package(mujoco_ros_testing_utils REQUIRED)
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(mujoco_ros_testing_utils REQUIRED)
endif()

# Helper function handling ROS 1/2 specific setup
function(add_mujoco_ros_integration_test TEST_NAME TEST_SOURCE)
  set(options)
  set(oneValueArgs ROS1_LAUNCH ROS2_LAUNCH)
  set(multiValueArgs EXTRA_LIBS)
  cmake_parse_arguments(TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" "${ARGN}")

  if(ROS_VERSION EQUAL 1)
    add_rostest_gtest(${TEST_NAME}
      ${TEST_ROS1_LAUNCH}
      ${TEST_SOURCE}
    )

    add_dependencies(${TEST_NAME}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )

    target_link_libraries(${TEST_NAME}
      mujoco_ros
      ${mujoco_ros_testing_utils_LIBRARIES}
      project_option
      project_warning
      ${TEST_EXTRA_LIBS}
    )

    target_include_directories(${TEST_NAME}
      PRIVATE
        ${mujoco_ros_testing_utils_INCLUDE_DIRS}
    )
  elseif(ROS_VERSION EQUAL 2)
    set(TEST_EXECUTABLE ${TEST_NAME})
    set(TEST_LAUNCH_NAME ${TEST_NAME}_launch)

    ament_add_gtest_executable(${TEST_EXECUTABLE}
      ${TEST_SOURCE}
    )

    target_link_libraries(${TEST_EXECUTABLE}
      mujoco_ros
      project_option
      project_warning
      ${TEST_EXTRA_LIBS}
    )

    ament_target_dependencies(${TEST_EXECUTABLE}
      mujoco_ros_testing_utils
    )

    # Register ROS 2 test launch file
    add_launch_test(${TEST_ROS2_LAUNCH}
      TARGET ${TEST_LAUNCH_NAME}
      ARGS "test_binary:=$<TARGET_FILE:${TEST_EXECUTABLE}>"
    )
  endif()
endfunction()

add_mujoco_ros_integration_test(
  mujoco_env_test
  mujoco_env_test.cpp
  ROS1_LAUNCH launch/mujoco_env.test
  ROS2_LAUNCH launch/mujoco_env.test.py
)

add_mujoco_ros_integration_test(
  ros_interface_test
  ros_interface_test.cpp
  ROS1_LAUNCH launch/ros_interface.test
  ROS2_LAUNCH launch/ros_interface.test.py
)

add_subdirectory(test_plugin)

if(ROS_VERSION EQUAL 2)
  # Forward pluginlib export vars from this directory to top-level scope.
  set(__PLUGINLIB_PLUGIN_CATEGORIES "${__PLUGINLIB_PLUGIN_CATEGORIES}" PARENT_SCOPE)
  set(__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros "${__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros}" PARENT_SCOPE)
endif()

add_mujoco_ros_integration_test(
  mujoco_ros_plugin_test
  mujoco_ros_plugin_test.cpp
  ROS1_LAUNCH launch/mujoco_ros_plugin.test
  ROS2_LAUNCH launch/mujoco_ros_plugin.test.py
  EXTRA_LIBS mujoco_testplugin
)

add_mujoco_ros_integration_test(
  mujoco_render_test
  mujoco_render_test.cpp
  ROS1_LAUNCH launch/mujoco_render.test
  ROS2_LAUNCH launch/mujoco_render.test.py
)

if(ROS_VERSION EQUAL 1)
  add_rostest(launch/python_bindings.test)
  add_rostest(launch/python_bindings_config.test)
  add_rostest(launch/python_bindings_plugin.test)
elseif(ROS_VERSION EQUAL 2)
  add_launch_test(launch/python_bindings.test.py
    TARGET python_bindings_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_test)
    add_dependencies(python_bindings_test pymujoco_ros)
  endif()
  add_launch_test(launch/python_bindings_config.test.py
    TARGET python_bindings_config_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_config_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_config_test)
    add_dependencies(python_bindings_config_test pymujoco_ros mujoco_testplugin)
  endif()
  add_launch_test(launch/python_bindings_plugin.test.py
    TARGET python_bindings_plugin_test
    ARGS
      "test_script:=${CMAKE_CURRENT_SOURCE_DIR}/python/python_bindings_plugin_test.py"
      "python_module_path:=$<TARGET_FILE_DIR:pymujoco_ros>"
      "python_package_path:=${PROJECT_SOURCE_DIR}/python/src"
  )
  if(TARGET python_bindings_plugin_test)
    add_dependencies(python_bindings_plugin_test pymujoco_ros mujoco_testplugin)
  endif()
endif()
