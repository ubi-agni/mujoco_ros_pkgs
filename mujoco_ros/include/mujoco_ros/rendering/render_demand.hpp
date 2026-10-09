#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/rendering/camera_descriptor.hpp>

namespace mujoco_ros::rendering {

class ConsumerId
{
public:
	explicit constexpr ConsumerId(std::uint64_t value = 0) : value_(value) {}
	constexpr std::uint64_t value() const { return value_; }
	friend constexpr bool operator==(ConsumerId lhs, ConsumerId rhs) { return lhs.value_ == rhs.value_; }
	friend constexpr bool operator!=(ConsumerId lhs, ConsumerId rhs) { return !(lhs == rhs); }

private:
	std::uint64_t value_;
};

enum class ConsumerMode
{
	kCadenced,
	kOneShot,
	kContinuous,
};

struct RenderPlan
{
	CameraId camera_id = 0;
	CameraDescriptor camera;
	PlaneMask planes = PlaneMask::kRgb;
	FrameGeneration frame_generation;
	std::chrono::nanoseconds evaluated_at{ 0 };
	std::vector<ConsumerId> consumers;
};

class DemandScheduler
{
public:
	ConsumerId RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence);
	ConsumerId RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence, CameraId camera);
	ConsumerId RegisterOneShotConsumer(const std::string &name);
	ConsumerId RegisterOneShotConsumer(const std::string &name, CameraId camera);
	ConsumerId RegisterContinuousConsumer(const std::string &name);
	ConsumerId RegisterContinuousConsumer(const std::string &name, CameraId camera);
	void UnregisterConsumer(ConsumerId consumer);
	void RequestOneShot(ConsumerId consumer);
	void SetEnabled(ConsumerId consumer, bool enabled);
	// Restrict which configured camera planes this consumer needs. Unset (default) means all
	// camera planes when the consumer is due — used by Python/continuous buffers.
	void SetConsumerPlanes(ConsumerId consumer, PlaneMask planes);

	RenderPlan Evaluate(std::chrono::nanoseconds simulation_time, CameraId camera) const;
	PlaneMask ResolvePlanes(const RenderPlan &plan, PlaneMask camera_planes) const;
	void MarkDelivered(const RenderPlan &plan, ConsumerId consumer);

private:
	struct ConsumerState
	{
		std::string name;
		ConsumerMode mode = ConsumerMode::kOneShot;
		std::chrono::nanoseconds cadence{ 0 };
		std::chrono::nanoseconds last_delivery{ 0 };
		bool one_shot_requested = false;
		CameraId camera_id      = 0;
		bool enabled            = true;
		std::optional<PlaneMask> planes;
	};

	std::uint64_t next_id_ = 1;
	std::unordered_map<std::uint64_t, ConsumerState> consumers_;
};

} // namespace mujoco_ros::rendering
