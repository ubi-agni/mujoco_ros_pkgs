#pragma once

#include <mujoco/mujoco.h>

namespace mujoco_ros::middleware {

/**
 * Outbound port: publishes simulation time (/clock). Takes the same sim time (mjtNum seconds)
 * that RosAPI::PublishSimTime consumes today.
 */
class ITimePublisher
{
public:
	virtual ~ITimePublisher() = default;

	virtual void PublishSimTime(mjtNum sim_time) = 0;
};

} // namespace mujoco_ros::middleware
