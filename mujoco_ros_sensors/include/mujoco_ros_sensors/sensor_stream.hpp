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
#include <stdexcept>
#include <string>
#include <utility>

namespace mujoco_ros::sensors {

/**
 * Runtime side of a sensor: owns publish/read behaviour for one SensorConfig.
 * The config stays the parameter bag; the stream only references it, so the
 * config must outlive the stream.
 */
static constexpr std::size_t SENSOR_PUBLISHER_QUEUE_SIZE = 1000;

#if MJR_ROS_VERSION == ROS_1
using AdvertiseNode = ros::NodeHandle;

template <typename MessageT>
ros::Publisher CreatePublisher(AdvertiseNode &node, const std::string &topic)
{
	return node.advertise<MessageT>(topic, SENSOR_PUBLISHER_QUEUE_SIZE, true);
}
#else
using AdvertiseNode = rclcpp_lifecycle::LifecycleNode::SharedPtr;

template <typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr CreatePublisher(AdvertiseNode &node, const std::string &topic)
{
	return node->create_publisher<MessageT>(topic, rclcpp::QoS(SENSOR_PUBLISHER_QUEUE_SIZE));
}
#endif

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
	 * Takes this stream off the hub for good (a compound claimed it). Its publishers are the caller's to
	 * drop; the stream stays usable as an internal value source for the compound.
	 */
	void Demote() { is_public_ = false; }

	/**
	 * True if the stream only reads model/data. Streams that write mjData (or
	 * otherwise need exclusive access) must override to false.
	 *
	 * Seam for future parallel stream evaluation: only observing streams may
	 * ever run concurrently. Nothing runs streams in parallel today.
	 */
	virtual bool ObservesOnly() const { return true; }

	/**
	 * Advertises the value topic (named like the stream) and, if `with_gt`, the `<name>_GT` topic on `config`.
	 * Only ever called for streams that stay public, so a claimed part never touches the topic graph.
	 */
	virtual void Advertise(AdvertiseNode & /*node*/, SensorConfig & /*config*/, bool /*with_gt*/) const
	{
		throw std::logic_error("sensor '" + name_ + "' has no ROS message type to advertise");
	}

	/// Rate check, subscriber gate and publish. Called per step from the owning provider's LastStage.
	virtual void MaybePublish(const mjModel *model, const mjData *data) = 0;

	/// Eager build of the current value for in-process (Python) reads; never waits for a subscriber.
	virtual std::any BuildValue(const mjModel *model, const mjData *data) = 0;

	/**
	 * Per-step shared cache: ROS publish and Python reads go through these, so one
	 * step yields one build (one noise sample set) no matter how many readers. Valid until
	 * InvalidateCache(), which the owning provider calls at the start of every LastStage and on
	 * Reset. Caller holds physics_thread_mutex_; the cache itself is unsynchronized.
	 * State changed outside a step (e.g. qvel written while paused) is not seen until the next step.
	 */
	const std::any &Value(const mjModel *model, const mjData *data)
	{
		if (!value_cache_.has_value()) {
			value_cache_ = BuildValue(model, data);
		}
		return value_cache_;
	}
	/// Throws (without caching) when HasGroundTruth() is false.
	const std::any &GroundTruth(const mjModel *model, const mjData *data)
	{
		if (!gt_cache_.has_value()) {
			gt_cache_ = BuildGroundTruth(model, data);
		}
		return gt_cache_;
	}
	void InvalidateCache()
	{
		value_cache_.reset();
		gt_cache_.reset();
	}

	/// True if a ground-truth value exists (never in eval mode: the agent must not see it).
	virtual bool HasGroundTruth() const { return false; }

	/// Eager ground-truth build for in-process reads. Throws std::runtime_error when HasGroundTruth() is false.
	virtual std::any BuildGroundTruth(const mjModel * /*model*/, const mjData * /*data*/)
	{
		throw std::runtime_error("sensor '" + name_ + "' has no ground truth: not available in eval mode");
	}

	/// Number of ROS messages actually published on the value / ground-truth topic since construction.
	uint64_t value_publish_count() const { return value_publish_count_.load(std::memory_order_acquire); }
	uint64_t gt_publish_count() const { return gt_publish_count_.load(std::memory_order_acquire); }

protected:
	// Atomic: the sim thread publishes while tests (and later Python) read from another thread.
	std::atomic<uint64_t> value_publish_count_{ 0 };
	std::atomic<uint64_t> gt_publish_count_{ 0 };

private:
	std::any value_cache_;
	std::any gt_cache_;
	std::string name_;
	const SensorConfig &config_;
	bool is_public_;
};

#if MJR_ROS_VERSION == ROS_1
template <typename MessageT>
void Publish(const ros::Publisher &publisher, const MessageT &message)
{
	publisher.publish(message);
}

template <typename MessageT>
bool HasSubscribers(const ros::Publisher &publisher)
{
	return publisher.getNumSubscribers() > 0;
}
#else
template <typename MessageT>
void Publish(const std::any &publisher, const MessageT &message)
{
	std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(publisher)->publish(message);
}

template <typename MessageT>
bool HasSubscribers(const std::any &publisher)
{
	// An empty any is the ground-truth publisher in eval mode: nobody can listen.
	if (!publisher.has_value()) {
		return false;
	}
	return std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(publisher)->get_subscription_count() > 0;
}
#endif

/**
 * Publishes the (possibly noisy) value and, outside eval mode, the ground
 * truth. Each channel is gated independently by subscribers and by the
 * optional update_rate against sim time, so an unwatched channel costs no
 * message build (and consumes no noise samples).
 */
template <typename MsgT>
class PublishingStream : public ISensorStream
{
public:
	PublishingStream(std::string name, const SensorConfig &config, bool eval_mode)
	    : ISensorStream(std::move(name), config, true), eval_mode_(eval_mode)
	{
	}

	void Advertise(AdvertiseNode &node, SensorConfig &config, bool with_gt) const override
	{
		config.RegisterPub(CreatePublisher<MsgT>(node, name()));
		if (with_gt) {
			config.RegisterGTPub(CreatePublisher<MsgT>(node, name() + "_GT"));
		}
	}

	void MaybePublish(const mjModel *model, const mjData *data) override
	{
		const double now = data->time;
		if (RateAllows(now, value_last_, value_has_last_) && HasSubscribers<MsgT>(config().value_pub)) {
			Publish(config().value_pub, std::any_cast<const MsgT &>(Value(model, data)));
			value_last_     = now;
			value_has_last_ = true;
			value_publish_count_.fetch_add(1, std::memory_order_release);
		}
		if (!eval_mode_ && RateAllows(now, gt_last_, gt_has_last_) && HasSubscribers<MsgT>(config().gt_pub)) {
			Publish(config().gt_pub, std::any_cast<const MsgT &>(GroundTruth(model, data)));
			gt_last_     = now;
			gt_has_last_ = true;
			gt_publish_count_.fetch_add(1, std::memory_order_release);
		}
	}

	std::any BuildValue(const mjModel *model, const mjData *data) override { return Noisy(model, data); }

	bool HasGroundTruth() const override { return !eval_mode_; }

	std::any BuildGroundTruth(const mjModel *model, const mjData *data) override
	{
		// Loud on purpose: silently returning noisy data as "truth" would corrupt evaluation.
		if (eval_mode_) {
			return ISensorStream::BuildGroundTruth(model, data);
		}
		return Truth(model, data);
	}

protected:
	virtual MsgT Noisy(const mjModel *model, const mjData *data) = 0;
	virtual MsgT Truth(const mjModel *model, const mjData *data) = 0;

	bool eval_mode_;

private:
	/**
	 * Only publishes advance last-publish time (not skipped steps), so a late
	 * subscriber on one channel is served at once. now < last means sim time was
	 * reset; waiting out the old last would stall the stream.
	 */
	bool RateAllows(double now, double &last, bool &has_last) const
	{
		const double rate = config().update_rate;
		if (rate <= 0.0 || !has_last) {
			return true;
		}
		if (now < last) {
			has_last = false;
			return true;
		}
		// Tolerance: sim time is a float sum of timesteps and lands just below the period otherwise.
		return now - last >= 1.0 / rate - 1e-6;
	}

	double value_last_   = 0.0;
	double gt_last_      = 0.0;
	bool value_has_last_ = false;
	bool gt_has_last_    = false;
};

} // namespace mujoco_ros::sensors
