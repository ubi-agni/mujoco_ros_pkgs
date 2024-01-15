add_library(mujoco_testplugin
  test_plugin.cpp
)
target_link_libraries(mujoco_testplugin
  project_option
  project_warning
  mujoco_ros
  ${catkin_LIBRARIES}
)

install(TARGETS mujoco_testplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  test_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test/test_plugin
)
