#include <mujoco_ros/detail/viewer_connection_state.hpp>

#include <utility>

namespace mujoco_ros {

EnvironmentLease::EnvironmentLease(std::shared_ptr<ViewerConnectionState> state, MujocoEnv *environment)
    : state_(std::move(state)), environment_(environment)
{
}

EnvironmentLease::EnvironmentLease(EnvironmentLease &&other) noexcept
    : state_(std::move(other.state_)), environment_(other.environment_)
{
	other.environment_ = nullptr;
}

EnvironmentLease &EnvironmentLease::operator=(EnvironmentLease &&other) noexcept
{
	if (this != &other) {
		if (state_) {
			state_->ReleaseEnvironmentLease();
		}
		state_             = std::move(other.state_);
		environment_       = other.environment_;
		other.environment_ = nullptr;
	}
	return *this;
}

EnvironmentLease::~EnvironmentLease()
{
	if (state_) {
		state_->ReleaseEnvironmentLease();
	}
}

MujocoEnv *EnvironmentLease::get() const noexcept
{
	return environment_;
}

EnvironmentLease::operator bool() const noexcept
{
	return environment_ != nullptr;
}

ViewerOperationLease::ViewerOperationLease(std::shared_ptr<ViewerConnectionState> state, Viewer *viewer)
    : state_(std::move(state)), viewer_(viewer)
{
}

ViewerOperationLease::ViewerOperationLease(ViewerOperationLease &&other) noexcept
    : state_(std::move(other.state_)), viewer_(other.viewer_)
{
	other.viewer_ = nullptr;
}

ViewerOperationLease &ViewerOperationLease::operator=(ViewerOperationLease &&other) noexcept
{
	if (this != &other) {
		if (state_) {
			state_->ReleaseViewerOperationLease();
		}
		state_        = std::move(other.state_);
		viewer_       = other.viewer_;
		other.viewer_ = nullptr;
	}
	return *this;
}

ViewerOperationLease::~ViewerOperationLease()
{
	if (state_) {
		state_->ReleaseViewerOperationLease();
	}
}

Viewer *ViewerOperationLease::get() const noexcept
{
	return viewer_;
}

ViewerOperationLease::operator bool() const noexcept
{
	return viewer_ != nullptr;
}

ViewerConnectionState::ViewerConnectionState(MujocoEnv *environment, Viewer *viewer)
    : environment_(environment), viewer_(viewer)
{
}

EnvironmentLease ViewerConnectionState::TryAcquireEnvironment()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (environment_closed_ || admission_closed_ || environment_ == nullptr) {
		return EnvironmentLease();
	}
	++environment_leases_;
	return EnvironmentLease(shared_from_this(), environment_);
}

bool ViewerConnectionState::TryActivateRenderLoop(std::thread::id owner)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (environment_closed_ || admission_closed_ || render_loop_active_ || viewer_ == nullptr) {
		return false;
	}
	render_loop_active_ = true;
	render_thread_id_   = owner;
	return true;
}

void ViewerConnectionState::FinishRenderLoop() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	render_loop_active_ = false;
	render_loop_ready_  = false;
	render_thread_id_   = {};
	cv_.notify_all();
}

void ViewerConnectionState::MarkRenderLoopReady() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	render_loop_ready_ = true;
}

bool ViewerConnectionState::RenderLoopReady() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return render_loop_ready_;
}

void ViewerConnectionState::RequestStop() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	stop_requested_ = true;
	cv_.notify_all();
}

bool ViewerConnectionState::StopRequested() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return stop_requested_;
}

bool ViewerConnectionState::IsRenderThread(std::thread::id id) const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return render_loop_active_ && render_thread_id_ == id;
}

void ViewerConnectionState::CloseEnvironment() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	environment_closed_ = true;
	admission_closed_   = true;
	stop_requested_     = true;
	cv_.notify_all();
}

void ViewerConnectionState::DetachEnvironment() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	environment_ = nullptr;
	cv_.notify_all();
}

void ViewerConnectionState::WaitForLeases()
{
	std::unique_lock<std::mutex> lock(mutex_);
	cv_.wait(lock, [this]() { return environment_leases_ == 0 && viewer_operation_leases_ == 0; });
}

void ViewerConnectionState::WaitForDrain(std::thread::id except_render_owner)
{
	std::unique_lock<std::mutex> lock(mutex_);
	cv_.wait(lock, [this, except_render_owner]() {
		const bool render_inactive = !render_loop_active_;
		const bool render_self =
		    render_loop_active_ && except_render_owner != std::thread::id{} && render_thread_id_ == except_render_owner;
		return environment_leases_ == 0 && viewer_operation_leases_ == 0 && (render_inactive || render_self);
	});
}

void ViewerConnectionState::WaitForViewerOperationLeases()
{
	std::unique_lock<std::mutex> lock(mutex_);
	cv_.wait(lock, [this]() { return viewer_operation_leases_ == 0; });
}

ViewerOperationLease ViewerConnectionState::TryAcquireViewerOperation()
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (viewer_ == nullptr) {
		return ViewerOperationLease();
	}
	++viewer_operation_leases_;
	return ViewerOperationLease(shared_from_this(), viewer_);
}

Viewer *ViewerConnectionState::viewer() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return viewer_;
}

void ViewerConnectionState::DetachViewer() noexcept
{
	std::unique_lock<std::mutex> lock(mutex_);
	if (viewer_ == nullptr) {
		return;
	}
	cv_.wait(lock, [this]() { return viewer_operation_leases_ == 0; });
	viewer_ = nullptr;
	cv_.notify_all();
}

void ViewerConnectionState::AcquireViewerOperationLease()
{
	std::lock_guard<std::mutex> lock(mutex_);
	++viewer_operation_leases_;
}

void ViewerConnectionState::ReleaseViewerOperationLease() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (viewer_operation_leases_ > 0) {
		--viewer_operation_leases_;
	}
	cv_.notify_all();
}

bool ViewerConnectionState::AdmissionOpen() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return !admission_closed_ && !environment_closed_;
}

bool ViewerConnectionState::EnvironmentClosed() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return environment_closed_;
}

bool ViewerConnectionState::RenderLoopActive() const noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	return render_loop_active_;
}

void ViewerConnectionState::ReleaseEnvironmentLease() noexcept
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (environment_leases_ > 0) {
		--environment_leases_;
	}
	cv_.notify_all();
}

} // namespace mujoco_ros
