#include <mujoco_ros/middleware/middleware_core.hpp>

namespace mujoco_ros::middleware {

MiddlewareCore::MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher)
    : config_source_(config_source), time_publisher_(time_publisher)
{
}

} // namespace mujoco_ros::middleware
