#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mujoco_ros::middleware {

/**
 * Outbound port: startup configuration reads.
 *
 * Mirrors the ROS 1 (`nh_->getParam`) / ROS 2 (`get_parameter`) reads performed during startup
 * (eval_mode, no_render, render_offscreen, headless, unpause, num_steps, num_mj_threads,
 * render_backpressure_policy, modelfile, wait_for_xml, mujoco_xml, realtime, plugin names, ...).
 * No new config keys. An unset key returns std::nullopt. Flavor implementations stay private.
 */
class IConfigSource
{
public:
	virtual ~IConfigSource() = default;

	virtual std::optional<bool> GetBool(const std::string &key) const                           = 0;
	virtual std::optional<int> GetInt(const std::string &key) const                             = 0;
	virtual std::optional<double> GetDouble(const std::string &key) const                       = 0;
	virtual std::optional<std::string> GetString(const std::string &key) const                  = 0;
	virtual std::optional<std::vector<std::string>> GetStringList(const std::string &key) const = 0;
};

} // namespace mujoco_ros::middleware
