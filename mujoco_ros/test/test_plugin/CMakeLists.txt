add_library(mujoco_testplugin SHARED
  test_plugin.cpp
)

if(ROS_VERSION EQUAL 1)
  target_link_libraries(mujoco_testplugin
    project_option
    project_warning
    mujoco_ros
    ${catkin_LIBRARIES}
  )
elseif(ROS_VERSION EQUAL 2)
  target_link_libraries(mujoco_testplugin
    project_option
    project_warning
    mujoco_ros
  )
endif()

if(ROS_VERSION EQUAL 1)
  install(TARGETS mujoco_testplugin
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES
    test_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/test_plugin
  )
elseif(ROS_VERSION EQUAL 2)
  pluginlib_export_plugin_description_file(mujoco_ros test_plugin.ros2.xml)

  # pluginlib accumulates export metadata in directory-scope vars.
  # Propagate to parent so ament_package() in top-level CMake can write
  # the ament-index resource for this plugin category.
  set(__PLUGINLIB_PLUGIN_CATEGORIES "${__PLUGINLIB_PLUGIN_CATEGORIES}" PARENT_SCOPE)
  set(__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros "${__PLUGINLIB_CATEGORY_CONTENT__mujoco_ros}" PARENT_SCOPE)

  install(TARGETS mujoco_testplugin
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    ARCHIVE DESTINATION lib
  )
endif()
