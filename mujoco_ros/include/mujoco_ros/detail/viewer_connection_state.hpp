#pragma once

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>

namespace mujoco_ros {

class MujocoEnv;
class Viewer;
class ViewerConnectionState;

class EnvironmentLease
{
public:
	EnvironmentLease() = default;
	EnvironmentLease(EnvironmentLease &&other) noexcept;
	EnvironmentLease &operator=(EnvironmentLease &&other) noexcept;
	~EnvironmentLease();

	EnvironmentLease(const EnvironmentLease &)            = delete;
	EnvironmentLease &operator=(const EnvironmentLease &) = delete;

	MujocoEnv *get() const noexcept;
	explicit operator bool() const noexcept;

private:
	friend class ViewerConnectionState;
	EnvironmentLease(std::shared_ptr<ViewerConnectionState> state, MujocoEnv *environment);

	std::shared_ptr<ViewerConnectionState> state_;
	MujocoEnv *environment_ = nullptr;
};

class ViewerOperationLease
{
public:
	ViewerOperationLease() = default;
	ViewerOperationLease(ViewerOperationLease &&other) noexcept;
	ViewerOperationLease &operator=(ViewerOperationLease &&other) noexcept;
	~ViewerOperationLease();

	ViewerOperationLease(const ViewerOperationLease &)            = delete;
	ViewerOperationLease &operator=(const ViewerOperationLease &) = delete;

	Viewer *get() const noexcept;
	explicit operator bool() const noexcept;

private:
	friend class ViewerConnectionState;
	ViewerOperationLease(std::shared_ptr<ViewerConnectionState> state, Viewer *viewer);

	std::shared_ptr<ViewerConnectionState> state_;
	Viewer *viewer_ = nullptr;
};

class ViewerConnectionState : public std::enable_shared_from_this<ViewerConnectionState>
{
public:
	explicit ViewerConnectionState(MujocoEnv *environment, Viewer *viewer);

	EnvironmentLease TryAcquireEnvironment();
	bool TryActivateRenderLoop(std::thread::id owner);
	void FinishRenderLoop() noexcept;
	void MarkRenderLoopReady() noexcept;
	bool RenderLoopReady() const noexcept;
	void RequestStop() noexcept;
	bool StopRequested() const noexcept;
	bool IsRenderThread(std::thread::id id) const noexcept;
	void CloseEnvironment() noexcept;
	void DetachEnvironment() noexcept;
	void WaitForLeases();
	void WaitForDrain(std::thread::id except_render_owner = std::thread::id{});
	void WaitForViewerOperationLeases();
	ViewerOperationLease TryAcquireViewerOperation();
	void RequestViewerExit() noexcept;

	Viewer *viewer() const noexcept;
	void DetachViewer() noexcept;

	void AcquireViewerOperationLease();
	void ReleaseViewerOperationLease() noexcept;

	bool AdmissionOpen() const noexcept;
	bool EnvironmentClosed() const noexcept;
	bool RenderLoopActive() const noexcept;

private:
	friend class EnvironmentLease;
	friend class ViewerOperationLease;
	friend class ConnectedViewersLease;

	void ReleaseEnvironmentLease() noexcept;

	mutable std::mutex mutex_;
	std::condition_variable cv_;

	MujocoEnv *environment_ = nullptr;
	Viewer *viewer_         = nullptr;

	bool environment_closed_ = false;
	bool admission_closed_   = false;
	bool stop_requested_     = false;

	std::thread::id render_thread_id_{};
	bool render_loop_active_ = false;
	// Distinct from render_loop_active_ (set as soon as the render thread claims
	// ownership, before ConnectViewer() has resolved): true only once this
	// viewer's RenderLoop has actually completed its initial ConnectViewer()
	// handshake and is about to enter its main frame loop. A reload's tail
	// (LoadWithModelAndData -> AcquireConnectedViewersLease -> viewer->Load())
	// must not hand an async load to a viewer that is still stuck inside its
	// own ConnectViewer() retry loop waiting for that same reload to finish --
	// nothing would ever service the async request, deadlocking both threads.
	bool render_loop_ready_ = false;

	std::size_t environment_leases_      = 0;
	std::size_t viewer_operation_leases_ = 0;
};

} // namespace mujoco_ros
