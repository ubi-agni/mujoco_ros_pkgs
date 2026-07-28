/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins */

#pragma once

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco/mujoco.h>
#include <sstream>

namespace mujoco_ros::util {

#if MJR_ROS_VERSION == ROS_1
static inline ros::Time toRosTime(mjtNum &time)
{
	ros::Time t = ros::Time(time);
	return t;
}
#else // MJR_ROS_VERSION == ROS_2
static inline rclcpp::Time toRosTime(mjtNum &time)
{
	rclcpp::Time t = rclcpp::Time(static_cast<uint64_t>(time * 1e9)); // convert to nanoseconds
	return t;
}
#endif

template <class T>
inline typename std::make_unsigned<T>::type as_unsigned(T x)
{
	return static_cast<typename std::make_unsigned<T>::type>(x);
}

static inline int jointName2id(mjModel *m, const std::string &joint_name,
                               const std::string &robot_namespace = std::string())
{
	int result = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
	if (result == -1 && !robot_namespace.empty()) {
		MJR_DEBUG_STREAM("Trying to find without namespace (" << joint_name.substr(robot_namespace.size()) << ")");
		result = mj_name2id(m, mjOBJ_JOINT, joint_name.substr(robot_namespace.size()).c_str());
	}
	return result;
}

// Helper function to convert a double array to a space-delimited string
static inline void arr_to_string(const mjtNum *arr, int size, std::string &str)
{
	str.clear();
	for (int i = 0; i < size; ++i) {
		str += std::to_string(arr[i]);
		if (i < size - 1) {
			str += " ";
		}
	}
}
// Helper function to set a bit in a flags int
static inline void bit_set_to(int &flags, int bit, bool value)
{
	if (value) {
		flags |= (1 << bit);
	} else {
		flags &= ~(1 << bit);
	}
}

// Helper function to read size values from a space-delimited string
static inline void set_from_string(mjtNum *vec, std::string str, uint8_t size)
{
	uint8_t count = 0;
	char *pch     = strtok(&str[0], " ");
	while (pch != nullptr) {
		if (count < size) {
			vec[count] = std::stod(pch);
			count++;
		} else {
			MJR_WARN_STREAM("Too many values in string '" << str << "' expected " << size << ". Ignoring the rest.");
		}
		pch = strtok(nullptr, " ");
	}
	if (count < size - 1) {
		MJR_WARN_STREAM("Too few values in string '" << str << "' expected " << size << ". Filling with zeros.");
		for (uint8_t i = count; i < size; i++) {
			vec[i] = 0;
		}
	}
}

template <typename T>
static inline std::string vector_to_string(const std::vector<T> &vec)
{
	std::ostringstream oss;
	for (size_t i = 0; i < vec.size(); ++i) {
		if constexpr (std::is_arithmetic_v<T>) { // Only handle arithmetic types (like double, float, int, etc.)
			oss << std::fixed << std::setprecision(2) << vec[i]; // Format to 2 decimal places
		} else {
			oss << vec[i]; // Handle non-arithmetic types
		}
		if (i != vec.size() - 1) {
			oss << ", "; // Add a comma except for the last element
		}
	}
	return oss.str();
}

} // namespace mujoco_ros::util
