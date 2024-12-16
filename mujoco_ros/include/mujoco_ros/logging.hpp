#pragma once

#include <mujoco_ros/ros_version.hpp>

///////////// ROS 1
#if MJR_ROS_VERSION == ROS_1

#include <ros/ros.h>
#define MJR_INFO(...) ROS_INFO(__VA_ARGS__)
#define MJR_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define MJR_WARN(...) ROS_WARN(__VA_ARGS__)
#define MJR_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define MJR_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define MJR_INFO_STREAM(args) ROS_INFO_STREAM(args)
#define MJR_DEBUG_STREAM(args) ROS_DEBUG_STREAM(args)
#define MJR_WARN_STREAM(args) ROS_WARN_STREAM(args)
#define MJR_ERROR_STREAM(args) ROS_ERROR_STREAM(args)
#define MJR_FATAL_STREAM(args) ROS_FATAL_STREAM(args)

#define MJR_INFO_COND(cond, ...) ROS_INFO_COND(cond, __VA_ARGS__)
#define MJR_DEBUG_COND(cond, ...) ROS_DEBUG_COND(cond, __VA_ARGS__)
#define MJR_WARN_COND(cond, ...) ROS_WARN_COND(cond, __VA_ARGS__)
#define MJR_ERROR_COND(cond, ...) ROS_ERROR_COND(cond, __VA_ARGS__)
#define MJR_FATAL_COND(cond, ...) ROS_FATAL_COND(cond, __VA_ARGS__)

#define MJR_INFO_NAMED(name, ...) ROS_INFO_NAMED(name, __VA_ARGS__)
#define MJR_DEBUG_NAMED(name, ...) ROS_DEBUG_NAMED(name, __VA_ARGS__)
#define MJR_WARN_NAMED(name, ...) ROS_WARN_NAMED(name, __VA_ARGS__)
#define MJR_ERROR_NAMED(name, ...) ROS_ERROR_NAMED(name, __VA_ARGS__)
#define MJR_FATAL_NAMED(name, ...) ROS_FATAL_NAMED(name, __VA_ARGS__)

#define MJR_INFO_STREAM_COND(cond, args) ROS_INFO_STREAM_COND(cond, args)
#define MJR_DEBUG_STREAM_COND(cond, args) ROS_DEBUG_STREAM_COND(cond, args)
#define MJR_WARN_STREAM_COND(cond, args) ROS_WARN_STREAM_COND(cond, args)
#define MJR_ERROR_STREAM_COND(cond, args) ROS_ERROR_STREAM_COND(cond, args)
#define MJR_FATAL_STREAM_COND(cond, args) ROS_FATAL_STREAM_COND(cond, args)

#define MJR_INFO_STREAM_NAMED(name, args) ROS_INFO_STREAM_NAMED(name, args)
#define MJR_DEBUG_STREAM_NAMED(name, args) ROS_DEBUG_STREAM_NAMED(name, args)
#define MJR_WARN_STREAM_NAMED(name, args) ROS_WARN_STREAM_NAMED(name, args)
#define MJR_ERROR_STREAM_NAMED(name, args) ROS_ERROR_STREAM_NAMED(name, args)
#define MJR_FATAL_STREAM_NAMED(name, args) ROS_FATAL_STREAM_NAMED(name, args)

#define MJR_INFO_COND_NAMED(cond, name, ...) ROS_INFO_COND_NAMED(cond, name, __VA_ARGS__)
#define MJR_DEBUG_COND_NAMED(cond, name, ...) ROS_DEBUG_COND_NAMED(cond, name, __VA_ARGS__)
#define MJR_WARN_COND_NAMED(cond, name, ...) ROS_WARN_COND_NAMED(cond, name, __VA_ARGS__)
#define MJR_ERROR_COND_NAMED(cond, name, ...) ROS_ERROR_COND_NAMED(cond, name, __VA_ARGS__)
#define MJR_FATAL_COND_NAMED(cond, name, ...) ROS_FATAL_COND_NAMED(cond, name, __VA_ARGS__)

#define MJR_INFO_STREAM_COND_NAMED(cond, name, args) ROS_INFO_STREAM_COND_NAMED(cond, name, args)
#define MJR_DEBUG_STREAM_COND_NAMED(cond, name, args) ROS_DEBUG_STREAM_COND_NAMED(cond, name, args)
#define MJR_WARN_STREAM_COND_NAMED(cond, name, args) ROS_WARN_STREAM_COND_NAMED(cond, name, args)
#define MJR_ERROR_STREAM_COND_NAMED(cond, name, args) ROS_ERROR_STREAM_COND_NAMED(cond, name, args)
#define MJR_FATAL_STREAM_COND_NAMED(cond, name, args) ROS_FATAL_STREAM_COND_NAMED(cond, name, args)

///////////// ROS 2
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>

#define MJR_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mujoco_server"), __VA_ARGS__)
#define MJR_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("mujoco_server"), __VA_ARGS__)
#define MJR_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mujoco_server"), __VA_ARGS__)
#define MJR_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("mujoco_server"), __VA_ARGS__)
#define MJR_FATAL(...) RCLCPP_FATAL(rclcpp::get_logger("mujoco_server"), __VA_ARGS__)

#define MJR_INFO_STREAM(args) RCLCPP_INFO_STREAM(rclcpp::get_logger("mujoco_server"), args)
#define MJR_DEBUG_STREAM(args) RCLCPP_DEBUG_STREAM(rclcpp::get_logger("mujoco_server"), args)
#define MJR_WARN_STREAM(args) RCLCPP_WARN_STREAM(rclcpp::get_logger("mujoco_server"), args)
#define MJR_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(rclcpp::get_logger("mujoco_server"), args)
#define MJR_FATAL_STREAM(args) RCLCPP_FATAL_STREAM(rclcpp::get_logger("mujoco_server"), args)

#define MJR_INFO_COND(cond, ...) RCLCPP_INFO_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, __VA_ARGS__);
#define MJR_DEBUG_COND(cond, ...) RCLCPP_DEBUG_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, __VA_ARGS__);
#define MJR_WARN_COND(cond, ...) RCLCPP_WARN_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, __VA_ARGS__);
#define MJR_ERROR_COND(cond, ...) RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, __VA_ARGS__);
#define MJR_FATAL_COND(cond, ...) RCLCPP_FATAL_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, __VA_ARGS__);

#define MJR_INFO_NAMED(name, ...) RCLCPP_INFO(rclcpp::get_logger(name), __VA_ARGS__)
#define MJR_DEBUG_NAMED(name, ...) RCLCPP_DEBUG(rclcpp::get_logger(name), __VA_ARGS__)
#define MJR_WARN_NAMED(name, ...) RCLCPP_WARN(rclcpp::get_logger(name), __VA_ARGS__)
#define MJR_ERROR_NAMED(name, ...) RCLCPP_ERROR(rclcpp::get_logger(name), __VA_ARGS__)
#define MJR_FATAL_NAMED(name, ...) RCLCPP_FATAL(rclcpp::get_logger(name), __VA_ARGS__)

#define MJR_INFO_STREAM_COND(cond, args) RCLCPP_INFO_STREAM_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, args);
#define MJR_DEBUG_STREAM_COND(cond, args) \
	RCLCPP_DEBUG_STREAM_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, args);
#define MJR_WARN_STREAM_COND(cond, args) RCLCPP_WARN_STREAM_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, args);
#define MJR_ERROR_STREAM_COND(cond, args) \
	RCLCPP_ERROR_STREAM_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, args);
#define MJR_FATAL_STREAM_COND(cond, args) \
	RCLCPP_FATAL_STREAM_EXPRESSION(rclcpp::get_logger("mujoco_server"), cond, args);

#define MJR_INFO_STREAM_NAMED(name, args) RCLCPP_INFO_STREAM(rclcpp::get_logger(name), args)
#define MJR_DEBUG_STREAM_NAMED(name, args) RCLCPP_DEBUG_STREAM(rclcpp::get_logger(name), args)
#define MJR_WARN_STREAM_NAMED(name, args) RCLCPP_WARN_STREAM(rclcpp::get_logger(name), args)
#define MJR_ERROR_STREAM_NAMED(name, args) RCLCPP_ERROR_STREAM(rclcpp::get_logger(name), args)
#define MJR_FATAL_STREAM_NAMED(name, args) RCLCPP_FATAL_STREAM(rclcpp::get_logger(name), args)

#define MJR_INFO_COND_NAMED(cond, name, ...) RCLCPP_INFO_EXPRESSION(rclcpp::get_logger(name), cond, __VA_ARGS__);
#define MJR_DEBUG_COND_NAMED(cond, name, ...) RCLCPP_DEBUG_EXPRESSION(rclcpp::get_logger(name), cond, __VA_ARGS__);
#define MJR_WARN_COND_NAMED(cond, name, ...) RCLCPP_WARN_EXPRESSION(rclcpp::get_logger(name), cond, __VA_ARGS__);
#define MJR_ERROR_COND_NAMED(cond, name, ...) RCLCPP_ERROR_EXPRESSION(rclcpp::get_logger(name), cond, __VA_ARGS__);
#define MJR_FATAL_COND_NAMED(cond, name, ...) RCLCPP_FATAL_EXPRESSION(rclcpp::get_logger(name), cond, __VA_ARGS__);

#define MJR_INFO_STREAM_COND_NAMED(cond, name, args) \
	RCLCPP_INFO_STREAM_EXPRESSION(rclcpp::get_logger(name), cond, args);
#define MJR_DEBUG_STREAM_COND_NAMED(cond, name, args) \
	RCLCPP_DEBUG_STREAM_EXPRESSION(rclcpp::get_logger(name), cond, args);
#define MJR_WARN_STREAM_COND_NAMED(cond, name, args) \
	RCLCPP_WARN_STREAM_EXPRESSION(rclcpp::get_logger(name), cond, args);
#define MJR_ERROR_STREAM_COND_NAMED(cond, name, args) \
	RCLCPP_ERROR_STREAM_EXPRESSION(rclcpp::get_logger(name), cond, args);
#define MJR_FATAL_STREAM_COND_NAMED(cond, name, args) \
	RCLCPP_FATAL_STREAM_EXPRESSION(rclcpp::get_logger(name), cond, args);

#endif
