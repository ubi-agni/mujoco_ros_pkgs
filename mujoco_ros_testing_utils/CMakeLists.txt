cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(mujoco_ros_testing_utils VERSION 0.10.0 LANGUAGES CXX)

if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    mujoco_ros_msgs
    roscpp
  )

  catkin_package(
    CATKIN_DEPENDS
      dynamic_reconfigure
      mujoco_ros_msgs
      roscpp
    INCLUDE_DIRS
      include
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(FILES
    assets/empty_world.xml
    assets/equality_world.xml
    assets/camera_world.xml
    assets/pendulum_world.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/assets
  )
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(mujoco_ros_msgs REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_export_include_directories(include)
  ament_export_dependencies(
    ament_index_cpp
    mujoco_ros_msgs
    rclcpp
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
  )

  install(FILES
    assets/empty_world.xml
    assets/equality_world.xml
    assets/camera_world.xml
    assets/pendulum_world.xml
    DESTINATION share/${PROJECT_NAME}/assets
  )

  ament_package()
endif()
