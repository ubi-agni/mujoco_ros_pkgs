/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#pragma once

#include <mujoco_ros/sensor_hub.hpp>
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

#include <mujoco/mujoco.h>

#include <any>
#include <atomic>
#include <cstdint>
#include <string>
#include <utility>

namespace mujoco_ros::sensors {

/**
 * Runtime side of a sensor: owns publish/read behaviour for one SensorConfig.
 * The config stays the parameter bag; the stream only references it, so the
 * config must outlive the stream.
 */
class ISensorStream : public mujoco_ros::SensorStreamBase
{
public:
	ISensorStream(std::string name, const SensorConfig &config, bool is_public)
	    : name_(std::move(name)), config_(config), is_public_(is_public)
	{
	}

	const std::string &name() const final { return name_; }
	bool is_public() const final { return is_public_; }
	const SensorConfig &config() const { return config_; }

	/**
	 * True if the stream only reads model/data. Streams that write mjData (or
	 * otherwise need exclusive access) must override to false.
	 *
	 * Seam for future parallel stream evaluation: only observing streams may
	 * ever run concurrently. Nothing runs streams in parallel today.
	 */
	virtual bool ObservesOnly() const { return true; }

	/// Rate check, subscriber gate and publish. Called per step from the owning provider's LastStage.
	virtual void MaybePublish(const mjModel *model, const mjData *data) = 0;

	/// Eager build of the current value for in-process (Python) reads; never waits for a subscriber.
	virtual std::any BuildValue(const mjModel *model, const mjData *data) = 0;

	/// Number of ROS messages actually published on the value / ground-truth topic since construction.
	uint64_t value_publish_count() const { return value_publish_count_.load(std::memory_order_acquire); }
	uint64_t gt_publish_count() const { return gt_publish_count_.load(std::memory_order_acquire); }

protected:
	// Atomic: the sim thread publishes while tests (and later Python) read from another thread.
	std::atomic<uint64_t> value_publish_count_{ 0 };
	std::atomic<uint64_t> gt_publish_count_{ 0 };

private:
	std::string name_;
	const SensorConfig &config_;
	bool is_public_;
};

} // namespace mujoco_ros::sensors
