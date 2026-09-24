#include <mujoco_ros/rendering/render_demand.hpp>

#include <algorithm>
#include <stdexcept>

namespace mujoco_ros::rendering {

ConsumerId DemandScheduler::RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence)
{
	return RegisterCadencedConsumer(name, cadence, CameraId(0));
}

ConsumerId DemandScheduler::RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence,
                                                     CameraId camera)
{
	if (cadence <= std::chrono::nanoseconds::zero()) {
		throw std::invalid_argument("consumer cadence must be positive");
	}
	const ConsumerId id(next_id_++);
	consumers_.emplace(id.value(), ConsumerState{ name, ConsumerMode::kCadenced, cadence, {}, false, camera, true });
	return id;
}

ConsumerId DemandScheduler::RegisterOneShotConsumer(const std::string &name)
{
	return RegisterOneShotConsumer(name, CameraId(0));
}

ConsumerId DemandScheduler::RegisterOneShotConsumer(const std::string &name, CameraId camera)
{
	const ConsumerId id(next_id_++);
	consumers_.emplace(id.value(), ConsumerState{ name, ConsumerMode::kOneShot, {}, {}, false, camera, true });
	return id;
}

ConsumerId DemandScheduler::RegisterContinuousConsumer(const std::string &name)
{
	return RegisterContinuousConsumer(name, CameraId(0));
}

ConsumerId DemandScheduler::RegisterContinuousConsumer(const std::string &name, CameraId camera)
{
	const ConsumerId id(next_id_++);
	consumers_.emplace(id.value(), ConsumerState{ name, ConsumerMode::kContinuous, {}, {}, false, camera, true });
	return id;
}

void DemandScheduler::UnregisterConsumer(ConsumerId consumer)
{
	const auto erased = consumers_.erase(consumer.value());
	if (erased == 0) {
		throw std::invalid_argument("unknown render consumer");
	}
}

void DemandScheduler::RequestOneShot(ConsumerId consumer)
{
	auto it = consumers_.find(consumer.value());
	if (it == consumers_.end()) {
		throw std::invalid_argument("unknown render consumer");
	}
	if (it->second.mode != ConsumerMode::kOneShot) {
		throw std::invalid_argument("one-shot demand requested for non-one-shot consumer");
	}
	it->second.one_shot_requested = true;
}

void DemandScheduler::SetEnabled(ConsumerId consumer, bool enabled)
{
	auto it = consumers_.find(consumer.value());
	if (it == consumers_.end()) {
		throw std::invalid_argument("unknown render consumer");
	}
	it->second.enabled = enabled;
}

RenderPlan DemandScheduler::Evaluate(std::chrono::nanoseconds simulation_time, CameraId camera) const
{
	RenderPlan plan;
	plan.camera_id    = camera;
	plan.evaluated_at = simulation_time;
	std::vector<std::uint64_t> ids;
	ids.reserve(consumers_.size());
	for (const auto &entry : consumers_) {
		ids.push_back(entry.first);
	}
	std::sort(ids.begin(), ids.end());
	for (const auto id : ids) {
		const auto &state = consumers_.at(id);
		if (!state.enabled || (state.camera_id != CameraId(0) && state.camera_id != camera)) {
			continue;
		}
		const bool due =
		    state.mode == ConsumerMode::kContinuous ||
		    (state.mode == ConsumerMode::kCadenced && simulation_time >= state.last_delivery + state.cadence) ||
		    (state.mode == ConsumerMode::kOneShot && state.one_shot_requested);
		if (due) {
			plan.consumers.emplace_back(id);
		}
	}
	return plan;
}

void DemandScheduler::MarkDelivered(const RenderPlan &plan, ConsumerId consumer)
{
	if (std::find(plan.consumers.begin(), plan.consumers.end(), consumer) == plan.consumers.end()) {
		throw std::invalid_argument("consumer was not selected by render plan");
	}
	auto it = consumers_.find(consumer.value());
	if (it == consumers_.end()) {
		throw std::invalid_argument("unknown render consumer");
	}
	if (it->second.mode == ConsumerMode::kOneShot) {
		it->second.one_shot_requested = false;
	} else if (it->second.mode == ConsumerMode::kCadenced) {
		it->second.last_delivery = plan.evaluated_at;
	}
}

} // namespace mujoco_ros::rendering
