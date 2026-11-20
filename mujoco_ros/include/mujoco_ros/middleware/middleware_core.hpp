#pragma once

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>

namespace mujoco_ros::middleware {

/**
 * Simulation Middleware Core (skeleton).
 *
 * Flavor-agnostic; talks to the outside only through IConfigSource and ITimePublisher.
 * Does not own the ports. The Simulation Control API and Plugin Host move in here in later tasks.
 */
class MiddlewareCore
{
public:
	MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher);
	~MiddlewareCore() = default;

	MiddlewareCore(const MiddlewareCore &)            = delete;
	MiddlewareCore &operator=(const MiddlewareCore &) = delete;

	IConfigSource &config_source() const { return config_source_; }
	ITimePublisher &time_publisher() const { return time_publisher_; }

private:
	IConfigSource &config_source_;
	ITimePublisher &time_publisher_;
};

} // namespace mujoco_ros::middleware
