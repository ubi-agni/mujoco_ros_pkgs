cmake_minimum_required(VERSION 3.14)
project(mujoco_ros_msgs)

if (DEFINED ENV{ROS_VERSION})
    set(ROS_VERSION $ENV{ROS_VERSION})
else()
    message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

if (ROS_VERSION EQUAL 1)
  include(Ros1)
else()
  include(Ros2)
endif()
