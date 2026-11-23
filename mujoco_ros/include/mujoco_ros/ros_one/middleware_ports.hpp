#pragma once

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/ros_one/ros_api.hpp>

#include <ros/ros.h>

#include <optional>
#include <string>
#include <vector>

namespace mujoco_ros::ros_one {

/// ROS 1 flavor adapter for the IConfigSource port: reads the private-namespace params MujocoEnv already uses.
/// Flavor-private; the Core only sees the port.
class RosConfigSource final : public middleware::IConfigSource
{
public:
	explicit RosConfigSource(ros::NodeHandle &nh) : nh_(nh) {}

	std::optional<bool> GetBool(const std::string &key) const override { return Read<bool>(key); }
	std::optional<int> GetInt(const std::string &key) const override { return Read<int>(key); }
	std::optional<double> GetDouble(const std::string &key) const override { return Read<double>(key); }
	std::optional<std::string> GetString(const std::string &key) const override { return Read<std::string>(key); }
	std::optional<std::vector<std::string>> GetStringList(const std::string &key) const override
	{
		return Read<std::vector<std::string>>(key);
	}

private:
	template <typename T>
	std::optional<T> Read(const std::string &key) const
	{
		T value;
		if (!nh_.getParam(key, value)) {
			return std::nullopt;
		}
		return value;
	}

	ros::NodeHandle &nh_;
};

/// ROS 1 flavor adapter for the ITimePublisher port: delegates to the existing /clock publish path.
class RosTimePublisher final : public middleware::ITimePublisher
{
public:
	explicit RosTimePublisher(RosAPI &api) : api_(api) {}
	void PublishSimTime(mjtNum sim_time) override { api_.PublishSimTime(sim_time); }

private:
	RosAPI &api_;
};

} // namespace mujoco_ros::ros_one
