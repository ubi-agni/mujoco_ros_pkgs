#pragma once

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/ros_two/ros_api.hpp>

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <vector>

namespace mujoco_ros::ros_two {

/// ROS 2 flavor adapter for the IConfigSource port: reads the node parameters MujocoEnv already uses.
/// Flavor-private; the Core only sees the port.
class RosConfigSource final : public middleware::IConfigSource
{
public:
	explicit RosConfigSource(rclcpp::Node &node) : node_(node) {}

	std::optional<bool> GetBool(const std::string &key) const override
	{
		return Read<bool>(key, rclcpp::ParameterType::PARAMETER_BOOL,
		                  [](const rclcpp::Parameter &p) { return p.as_bool(); });
	}
	std::optional<int> GetInt(const std::string &key) const override
	{
		return Read<int>(key, rclcpp::ParameterType::PARAMETER_INTEGER,
		                 [](const rclcpp::Parameter &p) { return static_cast<int>(p.as_int()); });
	}
	std::optional<double> GetDouble(const std::string &key) const override
	{
		rclcpp::Parameter param;
		if (!node_.get_parameter(key, param)) {
			return std::nullopt;
		}
		if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
			return static_cast<double>(param.as_int());
		}
		if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
			return param.as_double();
		}
		return std::nullopt;
	}
	std::optional<std::string> GetString(const std::string &key) const override
	{
		return Read<std::string>(key, rclcpp::ParameterType::PARAMETER_STRING,
		                         [](const rclcpp::Parameter &p) { return p.as_string(); });
	}
	std::optional<std::vector<std::string>> GetStringList(const std::string &key) const override
	{
		return Read<std::vector<std::string>>(key, rclcpp::ParameterType::PARAMETER_STRING_ARRAY,
		                                      [](const rclcpp::Parameter &p) { return p.as_string_array(); });
	}

private:
	template <typename T, typename Extract>
	std::optional<T> Read(const std::string &key, rclcpp::ParameterType type, Extract &&extract) const
	{
		rclcpp::Parameter param;
		if (!node_.get_parameter(key, param) || param.get_type() != type) {
			return std::nullopt;
		}
		return extract(param);
	}

	rclcpp::Node &node_;
};

/// ROS 2 flavor adapter for the ITimePublisher port: delegates to the existing /clock publish path.
class RosTimePublisher final : public middleware::ITimePublisher
{
public:
	explicit RosTimePublisher(RosAPI &api) : api_(api) {}
	void PublishSimTime(mjtNum sim_time) override { api_.PublishSimTime(sim_time); }

private:
	RosAPI &api_;
};

} // namespace mujoco_ros::ros_two
