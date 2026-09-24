#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/rendering/frame_boundary.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/rendering/render_backend_interface.hpp>
#include <mujoco_ros/rendering/render_demand.hpp>

namespace mujoco_ros::rendering {

class RenderCore
{
public:
	static constexpr std::size_t kDefaultMaxBytes = FrameBoundary::kDefaultMaxBytes;
	static constexpr std::size_t kDefaultMaxSlots = kBootstrapFrameSlotCapacity;
	explicit RenderCore(std::unique_ptr<IRenderBackend> backend, std::size_t max_slots = kDefaultMaxSlots,
	                    std::size_t max_bytes = kDefaultMaxBytes);
	~RenderCore();

	RenderCore(const RenderCore &)            = delete;
	RenderCore &operator=(const RenderCore &) = delete;

	ConsumerId RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence);
	ConsumerId RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence, CameraId camera);
	ConsumerId RegisterOneShotConsumer(const std::string &name);
	ConsumerId RegisterOneShotConsumer(const std::string &name, CameraId camera);
	ConsumerId RegisterContinuousConsumer(const std::string &name);
	ConsumerId RegisterContinuousConsumer(const std::string &name, CameraId camera);
	void UnregisterConsumer(ConsumerId consumer);
	void RequestOneShot(ConsumerId consumer);
	void SetConsumerEnabled(ConsumerId consumer, bool enabled);
	RenderPlan EvaluateDemand(std::chrono::nanoseconds simulation_time, CameraId camera) const;
	void MarkDelivered(const RenderPlan &plan, ConsumerId consumer);
	FrameStatus UpdateCameraVisualOptions(CameraId camera, const mjvOption &options);
	FrameStatus SetRenderBackpressurePolicy(RenderBackpressurePolicy policy);
	FrameStatus SetRenderBackpressurePolicy(const std::string &policy);
	RenderBackpressurePolicy GetRenderBackpressurePolicy() const;
	FrameStatus WaitForFrameCapacity(const RenderPlan &plan);
#ifdef MJR_BUILD_TESTING
	void SetCapacityWaitObserverForTesting(std::function<void()> observer);
#endif

	FrameStatus Reconfigure(ModelGeneration model_generation, FrameGeneration frame_generation,
	                        const FrameLayout &layout, const std::vector<CameraDescriptor> &cameras,
	                        std::size_t warm_slot_count = 0);
	void StartAcceptingSnapshots();
	FrameStatus SubmitSnapshot(RenderSnapshot snapshot, const RenderPlan &plan);
	FrameStatus SubmitSnapshot(std::shared_ptr<const RenderSnapshot> snapshot, const RenderPlan &plan);
	void StopAcceptingSnapshots();
	void RequestCancelRenderTurn();
	FrameStatus FinishOrCancelRenderTurn();
	std::uint64_t LastTurnCaptureId() const;
	std::optional<FrameLease> AcquireLatest(PlaneKind plane) const;
	std::optional<FrameLease> AcquireLatest(CameraId camera, PlaneKind plane) const;
	std::optional<FrameLease> AcquireLatest(std::uint64_t capture_id, CameraId camera, PlaneKind plane) const;
	std::vector<FrameLease> AcquireRecent(CameraId camera, PlaneKind plane, std::size_t max_count) const;
	FrameStatus Status() const;
	void Shutdown();

	FrameBoundary &frames() { return frame_boundary_; }
	const FrameBoundary &frames() const { return frame_boundary_; }

private:
	struct PendingTurn
	{
		std::shared_ptr<const RenderSnapshot> snapshot;
		RenderPlan plan;
		FrameGeneration frame_generation;
	};

	void RenderLoop();
	FrameStatus ToFrameStatus(const RenderStatus &status) const;

	std::unique_ptr<IRenderBackend> backend_;
	FrameBoundary frame_boundary_;
	DemandScheduler demand_;
	std::unordered_map<CameraId, CameraDescriptor> cameras_;
	mutable std::mutex mutex_;
	std::condition_variable condition_;
	std::condition_variable idle_condition_;
	std::optional<PendingTurn> pending_turn_;
	RenderConfiguration configuration_;
	ModelGeneration model_generation_;
	bool configured_           = false;
	bool backend_initialized_  = false;
	bool reinitialize_backend_ = false;
	bool resize_backend_       = false;
	bool accepting_snapshots_  = true;
	bool cancel_requested_     = false;
	bool rendering_            = false;
	bool shutdown_requested_   = false;
	std::atomic<RenderBackpressurePolicy> backpressure_policy_{ RenderBackpressurePolicy::kDrop };
	std::atomic<std::uint64_t> backpressure_policy_epoch_{ 0 };
	std::atomic_bool capacity_wait_cancelled_{ false };
#ifdef MJR_BUILD_TESTING
	// Keep storage unconditional: downstream consumers may compile without
	// MJR_BUILD_TESTING, and public object layout must remain identical.
	std::function<void()> capacity_wait_observer_;
#endif
	RenderStatus terminal_status_;
	std::optional<RenderStatus> completed_status_;
	std::uint64_t last_turn_capture_id_ = 0;
	std::thread render_thread_;
};

} // namespace mujoco_ros::rendering
