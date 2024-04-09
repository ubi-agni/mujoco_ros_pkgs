find_package(rostest REQUIRED)

add_rostest_gtest(mocap_test
  launch/mocap_test.test
  mocap_test.cpp
)

add_dependencies(mocap_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mocap_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
