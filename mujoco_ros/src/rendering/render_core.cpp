#include <mujoco_ros/rendering/render_core.hpp>

#include <chrono>
#include <limits>
#include <stdexcept>

namespace mujoco_ros::rendering {

namespace {

FrameStatus CapacityRequirements(const RenderPlan &plan, std::size_t &required_slots, std::size_t &required_bytes)
{
	required_slots = 0;
	required_bytes = 0;
	for (const auto plane : { PlaneKind::kRgb, PlaneKind::kDepth, PlaneKind::kSegmentation }) {
		if (!HasPlane(plan.planes, plane)) {
			continue;
		}
		const auto bytes = plan.camera.layout(plane).byte_length;
		if (required_bytes > std::numeric_limits<std::size_t>::max() - bytes) {
			return FrameStatus{ FrameStatusCode::kInvalidLayout, 0, plane, plan.frame_generation,
				                 "render plan frame byte capacity overflow" };
		}
		++required_slots;
		required_bytes += bytes;
	}
	return FrameStatus::Ok();
}

} // namespace

std::shared_ptr<const mjModel> CopyModel(const mjModel &source)
{
	auto *copy = mj_copyModel(nullptr, &source);
	if (copy == nullptr) {
		throw std::runtime_error("MuJoCo model copy failed");
	}
	return std::shared_ptr<const mjModel>(copy, mj_deleteModel);
}

RenderCore::RenderCore(std::unique_ptr<IRenderBackend> backend, std::size_t max_slots, std::size_t max_bytes)
    : backend_(std::move(backend)), frame_boundary_(max_slots, max_bytes)
{
	if (!backend_) {
		throw std::invalid_argument("RenderCore requires a backend");
	}
	render_thread_ = std::thread(&RenderCore::RenderLoop, this);
}

RenderCore::~RenderCore()
{
	Shutdown();
}

ConsumerId RenderCore::RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterCadencedConsumer(name, cadence);
}

ConsumerId RenderCore::RegisterCadencedConsumer(const std::string &name, std::chrono::nanoseconds cadence,
                                                CameraId camera)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterCadencedConsumer(name, cadence, camera);
}

ConsumerId RenderCore::RegisterOneShotConsumer(const std::string &name)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterOneShotConsumer(name);
}

ConsumerId RenderCore::RegisterOneShotConsumer(const std::string &name, CameraId camera)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterOneShotConsumer(name, camera);
}

ConsumerId RenderCore::RegisterContinuousConsumer(const std::string &name)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterContinuousConsumer(name);
}

ConsumerId RenderCore::RegisterContinuousConsumer(const std::string &name, CameraId camera)
{
	std::lock_guard<std::mutex> lock(mutex_);
	return demand_.RegisterContinuousConsumer(name, camera);
}

void RenderCore::UnregisterConsumer(ConsumerId consumer)
{
	std::lock_guard<std::mutex> lock(mutex_);
	demand_.UnregisterConsumer(consumer);
}

void RenderCore::RequestOneShot(ConsumerId consumer)
{
	std::lock_guard<std::mutex> lock(mutex_);
	demand_.RequestOneShot(consumer);
}

void RenderCore::SetConsumerEnabled(ConsumerId consumer, bool enabled)
{
	std::lock_guard<std::mutex> lock(mutex_);
	demand_.SetEnabled(consumer, enabled);
}

RenderPlan RenderCore::EvaluateDemand(std::chrono::nanoseconds simulation_time, CameraId camera) const
{
	std::lock_guard<std::mutex> lock(mutex_);
	auto plan             = demand_.Evaluate(simulation_time, camera);
	plan.frame_generation = configuration_.generation;
	const auto it         = cameras_.find(camera);
	if (it != cameras_.end()) {
		plan.camera = it->second;
		plan.planes = it->second.planes;
	}
	return plan;
}

void RenderCore::MarkDelivered(const RenderPlan &plan, ConsumerId consumer)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (plan.frame_generation != configuration_.generation) {
		throw std::invalid_argument("render plan frame generation is stale");
	}
	demand_.MarkDelivered(plan, consumer);
}

FrameStatus RenderCore::UpdateCameraVisualOptions(CameraId camera, const mjvOption &options)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (shutdown_requested_) {
		return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, configuration_.generation,
			                 "RenderCore is shut down" };
	}
	const auto it = cameras_.find(camera);
	if (it == cameras_.end()) {
		return FrameStatus{ FrameStatusCode::kFrameUnavailable, 0, std::nullopt, configuration_.generation,
			                 "camera visual options target an unknown camera" };
	}
	it->second.visual_options = options;
	return FrameStatus::Ok();
}

FrameStatus RenderCore::SetRenderBackpressurePolicy(RenderBackpressurePolicy policy)
{
	if (policy != RenderBackpressurePolicy::kDrop && policy != RenderBackpressurePolicy::kWaitForSlot) {
		return FrameStatus{ FrameStatusCode::kInvalidPolicy, 0, std::nullopt, FrameGeneration(0),
			                 "render backpressure policy is invalid" };
	}
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (shutdown_requested_) {
			return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, configuration_.generation,
				                 "RenderCore is shut down" };
		}
		if (backpressure_policy_.load(std::memory_order_acquire) != policy) {
			backpressure_policy_.store(policy, std::memory_order_release);
			backpressure_policy_epoch_.fetch_add(1, std::memory_order_acq_rel);
		}
	}
	frame_boundary_.NotifyCapacityWaiters();
	condition_.notify_all();
	return FrameStatus::Ok();
}

FrameStatus RenderCore::SetRenderBackpressurePolicy(const std::string &policy)
{
	const auto parsed = RenderBackpressurePolicyFromString(policy);
	if (!parsed.has_value()) {
		return FrameStatus{ FrameStatusCode::kInvalidPolicy, 0, std::nullopt, FrameGeneration(0),
			                 "render backpressure policy must be 'drop' or 'wait_for_slot'" };
	}
	return SetRenderBackpressurePolicy(*parsed);
}

RenderBackpressurePolicy RenderCore::GetRenderBackpressurePolicy() const
{
	return backpressure_policy_.load(std::memory_order_acquire);
}

FrameStatus RenderCore::WaitForFrameCapacity(const RenderPlan &plan)
{
	std::size_t required_slots = 0;
	std::size_t required_bytes = 0;
	const auto requirements    = CapacityRequirements(plan, required_slots, required_bytes);
	if (!requirements.ok()) {
		return requirements;
	}
	if (required_slots == 0) {
		return FrameStatus::Ok();
	}

	std::uint64_t policy_epoch = 0;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!accepting_snapshots_ || shutdown_requested_) {
			return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, configuration_.generation,
				                 "RenderCore is not accepting snapshots" };
		}
		if (!terminal_status_.ok()) {
			return ToFrameStatus(terminal_status_);
		}
		if (plan.frame_generation != configuration_.generation) {
			return FrameStatus{ FrameStatusCode::kStaleGeneration, 0, std::nullopt, configuration_.generation,
				                 "render plan frame generation is stale" };
		}
		if (backpressure_policy_.load(std::memory_order_acquire) == RenderBackpressurePolicy::kDrop) {
			return FrameStatus::Ok();
		}
		policy_epoch = backpressure_policy_epoch_.load(std::memory_order_acquire);
	}

#ifdef MJR_BUILD_TESTING
	std::function<void()> wait_observer;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		wait_observer           = std::move(capacity_wait_observer_);
		capacity_wait_observer_ = {};
	}
	frame_boundary_.SetCapacityWaitObserverForTesting(std::move(wait_observer));
#endif

	return frame_boundary_.WaitForCapacity(
	    plan.frame_generation, required_slots, required_bytes, [this, policy_epoch]() {
		    return capacity_wait_cancelled_.load(std::memory_order_acquire) ||
		           backpressure_policy_.load(std::memory_order_acquire) != RenderBackpressurePolicy::kWaitForSlot ||
		           backpressure_policy_epoch_.load(std::memory_order_acquire) != policy_epoch;
	    });
}

#ifdef MJR_BUILD_TESTING
void RenderCore::SetCapacityWaitObserverForTesting(std::function<void()> observer)
{
	std::lock_guard<std::mutex> lock(mutex_);
	capacity_wait_observer_ = std::move(observer);
}
#endif

FrameStatus RenderCore::Reconfigure(ModelGeneration model_generation, FrameGeneration frame_generation,
                                    const FrameLayout &layout, const std::vector<CameraDescriptor> &cameras,
                                    std::size_t warm_slot_count)
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (shutdown_requested_) {
		return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, frame_generation, "RenderCore is shut down" };
	}
	idle_condition_.wait(lock, [this]() {
		return shutdown_requested_ || (!pending_turn_.has_value() && !rendering_ && !cancel_requested_);
	});
	if (shutdown_requested_) {
		return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, frame_generation, "RenderCore is shut down" };
	}
	std::unordered_map<CameraId, CameraDescriptor> next_cameras;
	for (const auto &camera : cameras) {
		if (camera.id == 0) {
			return FrameStatus{ FrameStatusCode::kInvalidLayout, 0, std::nullopt, frame_generation,
				                 "camera id zero is reserved" };
		}
		if (!next_cameras.emplace(camera.id, camera).second) {
			return FrameStatus{ FrameStatusCode::kInvalidLayout, 0, std::nullopt, frame_generation,
				                 "camera ids must be unique" };
		}
	}
	const auto frame_status = frame_boundary_.Reconfigure(frame_generation, layout, warm_slot_count);
	if (!frame_status.ok()) {
		return frame_status;
	}
	const bool model_changed = model_generation_ != model_generation;
	const bool frame_changed = configuration_.generation != frame_generation;
	if (model_changed || frame_changed) {
		pending_turn_.reset();
	}
	model_generation_           = model_generation;
	configuration_.generation   = frame_generation;
	configuration_.frame_layout = layout;
	cameras_                    = std::move(next_cameras);
	configured_                 = true;
	accepting_snapshots_        = true;
	cancel_requested_           = false;
	capacity_wait_cancelled_.store(false, std::memory_order_release);
	if (model_changed) {
		reinitialize_backend_ = true;
		resize_backend_       = false;
	} else if (frame_changed) {
		resize_backend_ = true;
	}
	condition_.notify_all();
	return FrameStatus::Ok();
}

void RenderCore::StartAcceptingSnapshots()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!shutdown_requested_ && terminal_status_.ok()) {
		accepting_snapshots_ = true;
		capacity_wait_cancelled_.store(false, std::memory_order_release);
	}
}

FrameStatus RenderCore::SubmitSnapshot(RenderSnapshot snapshot, const RenderPlan &plan)
{
	return SubmitSnapshot(std::make_shared<const RenderSnapshot>(std::move(snapshot)), plan);
}

FrameStatus RenderCore::SubmitSnapshot(std::shared_ptr<const RenderSnapshot> snapshot, const RenderPlan &plan)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!accepting_snapshots_) {
		return FrameStatus{ FrameStatusCode::kStopped, 0, std::nullopt, configuration_.generation,
			                 "RenderCore is not accepting snapshots" };
	}
	if (!terminal_status_.ok()) {
		return ToFrameStatus(terminal_status_);
	}
	if (!snapshot || snapshot->model_generation != model_generation_) {
		return FrameStatus{ FrameStatusCode::kStaleGeneration, 0, std::nullopt, configuration_.generation,
			                 "render snapshot model generation is stale" };
	}
	if (plan.frame_generation != configuration_.generation) {
		return FrameStatus{ FrameStatusCode::kStaleGeneration, 0, std::nullopt, configuration_.generation,
			                 "render plan frame generation is stale" };
	}
	if (!snapshot->valid()) {
		return FrameStatus{ FrameStatusCode::kFrameUnavailable, 0, std::nullopt, configuration_.generation,
			                 "render snapshot is incomplete" };
	}
	if (plan.consumers.empty()) {
		return FrameStatus::Ok();
	}
	if (plan.camera.id == 0) {
		const auto it = cameras_.find(plan.camera_id);
		if (it == cameras_.end()) {
			return FrameStatus{ FrameStatusCode::kFrameUnavailable, 0, std::nullopt, configuration_.generation,
				                 "render plan camera is not configured" };
		}
	}
	RenderPlan queued_plan = plan;
	if (queued_plan.camera.id == 0) {
		queued_plan.camera = cameras_.at(queued_plan.camera_id);
		queued_plan.planes = queued_plan.camera.planes;
	}
	if (pending_turn_.has_value()) {
		return FrameStatus{
			FrameStatusCode::kBusy, 0, std::nullopt, configuration_.generation,
			"queued render turn already exists (queued camera_id=" + std::to_string(pending_turn_->plan.camera_id) +
			    ", queued simulation_time_ns=" + std::to_string(pending_turn_->snapshot->simulation_time_ns) +
			    "; submitted camera_id=" + std::to_string(queued_plan.camera_id) +
			    ", submitted simulation_time_ns=" + std::to_string(snapshot->simulation_time_ns) + ")"
		};
	}
	pending_turn_ = PendingTurn{ std::move(snapshot), std::move(queued_plan), configuration_.generation };
	condition_.notify_all();
	return FrameStatus::Ok();
}

void RenderCore::StopAcceptingSnapshots()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		accepting_snapshots_ = false;
		capacity_wait_cancelled_.store(true, std::memory_order_release);
	}
	condition_.notify_all();
	frame_boundary_.NotifyCapacityWaiters();
}

void RenderCore::RequestCancelRenderTurn()
{
	std::lock_guard<std::mutex> lock(mutex_);
	capacity_wait_cancelled_.store(true, std::memory_order_release);
	if (shutdown_requested_ || !render_thread_.joinable()) {
		// Reload teardown may cancel after Shutdown() joined the render thread; clear
		// synchronously because no RenderLoop consumer remains to observe cancel_requested_.
		cancel_requested_ = false;
		pending_turn_.reset();
		idle_condition_.notify_all();
		return;
	}
	cancel_requested_ = true;
	condition_.notify_all();
	frame_boundary_.NotifyCapacityWaiters();
}

FrameStatus RenderCore::FinishOrCancelRenderTurn()
{
	std::unique_lock<std::mutex> lock(mutex_);
	idle_condition_.wait(lock, [this]() { return !pending_turn_.has_value() && !rendering_ && !cancel_requested_; });
	if (!terminal_status_.ok()) {
		return ToFrameStatus(terminal_status_);
	}
	if (completed_status_.has_value()) {
		const auto status = ToFrameStatus(*completed_status_);
		completed_status_.reset();
		return status;
	}
	return FrameStatus::Ok();
}

std::uint64_t RenderCore::LastTurnCaptureId() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return last_turn_capture_id_;
}

std::optional<FrameLease> RenderCore::AcquireLatest(PlaneKind plane) const
{
	return frame_boundary_.AcquireLatest(plane);
}

std::optional<FrameLease> RenderCore::AcquireLatest(CameraId camera, PlaneKind plane) const
{
	return frame_boundary_.AcquireLatest(camera, plane);
}

std::optional<FrameLease> RenderCore::AcquireLatest(std::uint64_t capture_id, CameraId camera, PlaneKind plane) const
{
	return frame_boundary_.AcquireLatest(capture_id, camera, plane);
}

std::vector<FrameLease> RenderCore::AcquireRecent(CameraId camera, PlaneKind plane, std::size_t max_count) const
{
	return frame_boundary_.AcquireRecent(camera, plane, max_count);
}

FrameStatus RenderCore::Status() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return ToFrameStatus(terminal_status_);
}

void RenderCore::Shutdown()
{
	if (!render_thread_.joinable()) {
		return;
	}
	{
		std::lock_guard<std::mutex> lock(mutex_);
		accepting_snapshots_ = false;
		shutdown_requested_  = true;
		cancel_requested_    = true;
		capacity_wait_cancelled_.store(true, std::memory_order_release);
	}
	condition_.notify_all();
	frame_boundary_.NotifyCapacityWaiters();
	render_thread_.join();
}

FrameStatus RenderCore::ToFrameStatus(const RenderStatus &status) const
{
	if (status.ok()) {
		return FrameStatus::Ok();
	}
	const auto code =
	    status.code == RenderStatusCode::kStaleModelGeneration        ? FrameStatusCode::kStaleGeneration :
	    status.code == RenderStatusCode::kStopped                     ? FrameStatusCode::kStopped :
	    status.code == RenderStatusCode::kNotInitialized              ? FrameStatusCode::kNotInitialized :
	    status.code == RenderStatusCode::kFrameUnavailable            ? FrameStatusCode::kFrameUnavailable :
	    status.code == RenderStatusCode::kFrameSlotsExhausted         ? FrameStatusCode::kFrameSlotsExhausted :
	    status.code == RenderStatusCode::kGenerationCapacityExhausted ? FrameStatusCode::kGenerationCapacityExhausted :
	    status.code == RenderStatusCode::kBackendUnavailable          ? FrameStatusCode::kBackendUnavailable :
	    status.code == RenderStatusCode::kBackendFailure              ? FrameStatusCode::kBackendFailure :
	                                                                    FrameStatusCode::kTerminalError;
	return FrameStatus{ code, 0, std::nullopt, configuration_.generation, status.message };
}

void RenderCore::RenderLoop()
{
	for (;;) {
		std::optional<PendingTurn> turn;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			condition_.wait(lock,
			                [this]() { return shutdown_requested_ || pending_turn_.has_value() || cancel_requested_; });
			if (shutdown_requested_ && !pending_turn_.has_value()) {
				cancel_requested_     = false;
				last_turn_capture_id_ = 0;
				idle_condition_.notify_all();
				break;
			}
			if (cancel_requested_) {
				pending_turn_.reset();
				cancel_requested_     = false;
				last_turn_capture_id_ = 0;
				idle_condition_.notify_all();
				continue;
			}
			turn = std::move(pending_turn_);
			pending_turn_.reset();
			rendering_ = true;
		}

		RenderStatus result = RenderStatus::Ok();
		RenderConfiguration configuration;
		bool reinitialize_backend     = false;
		bool resize_backend           = false;
		bool stale_turn               = false;
		bool backend_initialized      = false;
		std::uint64_t turn_capture_id = 0;
		try {
			{
				std::lock_guard<std::mutex> lock(mutex_);
				configuration = configuration_;
				stale_turn    = turn->snapshot->model_generation != model_generation_ ||
				             turn->frame_generation != configuration_.generation;
				reinitialize_backend = reinitialize_backend_;
				resize_backend       = resize_backend_;
				if (!stale_turn) {
					reinitialize_backend_ = false;
					resize_backend_       = false;
				}
				backend_initialized = backend_initialized_;
			}
			if (!stale_turn && reinitialize_backend && backend_initialized) {
				backend_->ShutdownOnRenderThread();
				backend_initialized = false;
			}
			if (!stale_turn && !backend_initialized) {
				result              = backend_->Initialize(*turn->snapshot->model, configuration);
				backend_initialized = result.ok();
			} else if (!stale_turn && resize_backend) {
				result = backend_->Resize(configuration);
			}
		} catch (const std::exception &error) {
			result = RenderStatus::Failure(RenderStatusCode::kBackendFailure, error.what());
		}

		if (!stale_turn && result.ok()) {
			try {
				const auto &camera = turn->plan.camera;
				const auto stamp   = frame_boundary_.BeginCapture(turn->snapshot->model_generation,
				                                                  turn->snapshot->simulation_time_ns, camera.id);
				turn_capture_id    = stamp.capture_id;
				result             = backend_->Render(*turn->snapshot, camera, turn->plan.planes, frame_boundary_);
			} catch (const std::exception &error) {
				result = ClassifyCaptureException(error);
			}
		}

		{
			std::lock_guard<std::mutex> lock(mutex_);
			rendering_        = false;
			completed_status_ = result;
			if (!stale_turn) {
				backend_initialized_ = backend_initialized;
			}
			if (cancel_requested_) {
				pending_turn_.reset();
				cancel_requested_     = false;
				last_turn_capture_id_ = 0;
			} else if (stale_turn || IsContextIntegrityFailure(result)) {
				last_turn_capture_id_ = 0;
			} else {
				last_turn_capture_id_ = turn_capture_id;
			}
			if (!stale_turn && IsContextIntegrityFailure(result)) {
				terminal_status_ = result;
			}
			idle_condition_.notify_all();
		}
	}
	backend_->ShutdownOnRenderThread();
	{
		std::lock_guard<std::mutex> lock(mutex_);
		backend_initialized_  = false;
		rendering_            = false;
		cancel_requested_     = false;
		last_turn_capture_id_ = 0;
		idle_condition_.notify_all();
	}
}

} // namespace mujoco_ros::rendering
