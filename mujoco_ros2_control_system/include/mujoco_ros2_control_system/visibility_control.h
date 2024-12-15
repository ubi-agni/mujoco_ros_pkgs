#ifndef MUJOCO_ROS2_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_
#define MUJOCO_ROS2_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define MUJOCO_ROS2_CONTROL_SYSTEM_EXPORT __attribute__ ((dllexport))
    #define MUJOCO_ROS2_CONTROL_SYSTEM_IMPORT __attribute__ ((dllimport))
  #else
    #define MUJOCO_ROS2_CONTROL_SYSTEM_EXPORT __declspec(dllexport)
    #define MUJOCO_ROS2_CONTROL_SYSTEM_IMPORT __declspec(dllimport)
  #endif
  #ifdef MUJOCO_ROS2_CONTROL_SYSTEM_BUILDING_LIBRARY
    #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC MUJOCO_ROS2_CONTROL_SYSTEM_EXPORT
  #else
    #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC MUJOCO_ROS2_CONTROL_SYSTEM_IMPORT
  #endif
  #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC_TYPE MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC
  #define MUJOCO_ROS2_CONTROL_SYSTEM_LOCAL
#else
  #define MUJOCO_ROS2_CONTROL_SYSTEM_EXPORT __attribute__ ((visibility("default")))
  #define MUJOCO_ROS2_CONTROL_SYSTEM_IMPORT
  #if __GNUC__ >= 4
    #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC __attribute__ ((visibility("default")))
    #define MUJOCO_ROS2_CONTROL_SYSTEM_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC
    #define MUJOCO_ROS2_CONTROL_SYSTEM_LOCAL
  #endif
  #define MUJOCO_ROS2_CONTROL_SYSTEM_PUBLIC_TYPE
#endif

#endif  // MUJOCO_ROS2_CONTROL_SYSTEM__VISIBILITY_CONTROL_H_
