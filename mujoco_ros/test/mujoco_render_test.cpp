/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor Neura Robotics nor the names of their
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David P. Leins */

#include <gtest/gtest.h>

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <boost/function.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#endif

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/util.hpp>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <future>
#include <functional>
#include <mutex>
#include <thread>
#include <array>
#include <cerrno>
#include <csignal>
#include <limits.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <mujoco_ros/rendering/render_core.hpp>
#include <mujoco_ros/rendering/render_snapshot.hpp>
#include <mujoco_ros/rendering/frame_capacity.hpp>
#ifdef MJR_BUILD_TESTING
#include <mujoco_ros/offscreen_python_buffer_test_access.hpp>
#endif

extern char **environ;

namespace mujoco_ros::rendering {
class OffscreenCameraTestAccess
{
public:
	static void SetHooks(OffscreenCamera &camera, std::function<void()> publication,
	                     std::function<void()> publication_end, std::function<void()> reset)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_test_hook_     = std::move(publication);
		camera.publication_end_test_hook_ = std::move(publication_end);
		camera.reset_test_hook_           = std::move(reset);
	}

	static void SetWorkerHook(OffscreenCamera &camera, std::function<void()> worker_publication)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.worker_publication_hook_ = std::move(worker_publication);
	}

	static void SetEnqueueHook(OffscreenCamera &camera, std::function<void()> publication_enqueue)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_enqueue_test_hook_ = std::move(publication_enqueue);
	}

	static void ClearTestHooks(OffscreenCamera &camera)
	{
		std::lock_guard<std::mutex> lock(camera.test_hook_mutex_);
		camera.publication_test_hook_               = {};
		camera.publication_enqueue_test_hook_       = {};
		camera.worker_publication_hook_             = {};
		camera.publication_publish_entry_test_hook_ = {};
		camera.publication_end_test_hook_           = {};
		camera.reset_test_hook_                     = {};
	}

	static bool PublicationWorkerActive(const OffscreenCamera &camera)
	{
		if (!camera.publication_queue_) {
			return false;
		}
		return !camera.publication_queue_->worker_idle();
	}

	static void WaitUntilPublicationWorkerIdle(OffscreenCamera &camera)
	{
		if (camera.publication_queue_) {
			camera.publication_queue_->WaitUntilWorkerIdle();
		}
	}

	static bool TryLockPublicationMutex(OffscreenCamera &camera) { return camera.publication_mutex_.try_lock(); }

	static void UnlockPublicationMutex(OffscreenCamera &camera) { camera.publication_mutex_.unlock(); }

	static bool HasActiveRenderCore(const OffscreenCamera &camera, const std::shared_ptr<RenderCore> &core)
	{
		std::lock_guard<std::mutex> lock(camera.descriptor_mutex_);
		return camera.active_render_core_.lock() == core;
	}

	static void FailNextRetirement(OffscreenCamera &camera) { camera.retirement_failure_once_.store(true); }
};
} // namespace mujoco_ros::rendering

namespace {

bool run_render_teardown_child = false;

TEST(RosTimestamp, PreservesIntegerNanoseconds)
{
	constexpr std::int64_t expected_nanoseconds = 2093999999;
	const auto timestamp                        = util::toRosTime(expected_nanoseconds);
#if MJR_ROS_VERSION == ROS_1
	EXPECT_EQ(static_cast<std::int64_t>(timestamp.toNSec()), expected_nanoseconds);
#else
	EXPECT_EQ(timestamp.nanoseconds(), expected_nanoseconds);
#endif
}

class ChildProcessReaper
{
public:
	explicit ChildProcessReaper(pid_t pid) : pid_(pid) {}

	~ChildProcessReaper()
	{
		if (!reaped_) {
			kill(pid_, SIGKILL);
			while (waitpid(pid_, nullptr, 0) == -1 && errno == EINTR) {
			}
		}
	}

	void MarkReaped() { reaped_ = true; }

private:
	pid_t pid_;
	bool reaped_ = false;
};

class ThreadJoinGuard
{
public:
	explicit ThreadJoinGuard(std::thread &thread) : thread_(thread) {}

	~ThreadJoinGuard()
	{
		if (thread_.joinable()) {
			thread_.join();
		}
	}

	ThreadJoinGuard(const ThreadJoinGuard &)            = delete;
	ThreadJoinGuard &operator=(const ThreadJoinGuard &) = delete;

private:
	std::thread &thread_;
};

} // namespace

int main(int argc, char **argv)
{
	for (int index = 1; index < argc; ++index) {
		if (std::string(argv[index]) == "--render-teardown-child") {
			run_render_teardown_child = true;
		}
	}
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_render_test", ros::init_options::AnonymousName);

	// Uncomment to enable debug output (useful for debugging failing tests)
	// ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	// ros::console::notifyLoggerLevelsChanged();

	// Create spinner to communicate with ROS
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;
	int ret = RUN_ALL_TESTS();

	// Stop spinner and shutdown ROS before returning
	spinner.stop();
	ros::shutdown();
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::init(argc, argv);
	::testing::InitGoogleTest(&argc, argv);
	int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
#endif
	return ret;
}

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

template <typename T, typename = void>
struct HasLegacyRequestPending : std::false_type
{};
template <typename T>
struct HasLegacyRequestPending<T, std::void_t<decltype(std::declval<T &>().request_pending)>> : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyCondRenderRequest : std::false_type
{};
template <typename T>
struct HasLegacyCondRenderRequest<T, std::void_t<decltype(std::declval<T &>().cond_render_request)>> : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyPendingRequestWaiters : std::false_type
{};
template <typename T>
struct HasLegacyPendingRequestWaiters<T, std::void_t<decltype(std::declval<T &>().pending_request_waiters)>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyRenderRequestWaiters : std::false_type
{};
template <typename T>
struct HasLegacyRenderRequestWaiters<T, std::void_t<decltype(std::declval<T &>().render_request_waiters)>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyShutdownExitObservers : std::false_type
{};
template <typename T>
struct HasLegacyShutdownExitObservers<T, std::void_t<decltype(std::declval<T &>().shutdown_exit_observers)>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyPauseShutdownExit : std::false_type
{};
template <typename T>
struct HasLegacyPauseShutdownExit<T, std::void_t<decltype(std::declval<T &>().pause_shutdown_exit)>> : std::true_type
{};

TEST(LegacyCompletionSignaling, TransportStateHasNoObsoleteCoordination)
{
	EXPECT_FALSE((HasLegacyRequestPending<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasLegacyCondRenderRequest<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasLegacyPendingRequestWaiters<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasLegacyRenderRequestWaiters<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasLegacyShutdownExitObservers<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasLegacyPauseShutdownExit<CameraPublicationTransport>::value));
}

template <typename T, typename = void>
struct HasDuplicateTransportRenderCore : std::false_type
{};
template <typename T>
struct HasDuplicateTransportRenderCore<T, std::void_t<decltype(std::declval<T &>().render_core)>> : std::true_type
{};

template <typename T, typename = void>
struct HasDuplicateTransportFrameGeneration : std::false_type
{};
template <typename T>
struct HasDuplicateTransportFrameGeneration<T, std::void_t<decltype(std::declval<T &>().frame_generation)>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasRenderSubmissionFlightAdmission : std::false_type
{};
template <typename T>
struct HasRenderSubmissionFlightAdmission<T, std::void_t<decltype(&T::BeginRenderSubmissionFlight)>> : std::true_type
{};

template <typename T, typename = void>
struct HasWaitForRenderSubmissionIdle : std::false_type
{};
template <typename T>
struct HasWaitForRenderSubmissionIdle<T, std::void_t<decltype(&T::WaitForRenderSubmissionIdle)>> : std::true_type
{};

template <typename T, typename = void>
struct HasCloseRenderSubmissionAdmission : std::false_type
{};
template <typename T>
struct HasCloseRenderSubmissionAdmission<T, std::void_t<decltype(&T::CloseRenderSubmissionAdmission)>> : std::true_type
{};

TEST(LegacyRenderOrchestration, TransportStateHasNoDuplicateRenderCoreHandle)
{
	EXPECT_FALSE((HasDuplicateTransportRenderCore<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasDuplicateTransportFrameGeneration<CameraPublicationTransport>::value));
	EXPECT_FALSE((HasRenderSubmissionFlightAdmission<MujocoEnv>::value));
	EXPECT_FALSE((HasWaitForRenderSubmissionIdle<MujocoEnv>::value));
	EXPECT_FALSE((HasCloseRenderSubmissionAdmission<MujocoEnv>::value));
}

template <typename T, typename = void>
struct HasGetOffscreenContextAccessor : std::false_type
{};
template <typename T>
struct HasGetOffscreenContextAccessor<T, std::void_t<decltype(std::declval<T &>().getOffscreenContext())>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasLegacyOffscreenTransportMember : std::false_type
{};
template <typename T>
struct HasLegacyOffscreenTransportMember<T, std::void_t<decltype(std::declval<T &>().offscreen_)>> : std::true_type
{};

template <typename T, typename = void>
struct HasBenchmarkInlinePublicationHook : std::false_type
{};
template <typename T>
struct HasBenchmarkInlinePublicationHook<T, std::void_t<decltype(std::declval<T &>().benchmark_inline_publication_)>>
    : std::true_type
{};

template <typename T, typename = void>
struct HasShouldRenderCadenceGate : std::false_type
{};
template <typename T>
struct HasShouldRenderCadenceGate<
    T, std::void_t<decltype(std::declval<const T &>().ShouldRender(std::declval<roscpp::Time>()))>> : std::true_type
{};

TEST(LegacyTransportNaming, FixtureHasNoGetOffscreenContextAccessor)
{
	EXPECT_FALSE((HasGetOffscreenContextAccessor<MujocoEnvTestWrapper>::value));
}

TEST(LegacyTransportNaming, EnvHasNoLegacyOffscreenTransportMember)
{
	EXPECT_FALSE((HasLegacyOffscreenTransportMember<MujocoEnv>::value));
}

TEST(LegacyTransportNaming, EnvOwnsCameraPublicationTransport)
{
	static_assert(std::is_same_v<
	              std::remove_pointer_t<decltype(std::declval<MujocoEnvTestWrapper>().getCameraPublicationTransport())>,
	              CameraPublicationTransport>);
}

TEST(LegacyProductionHooks, OffscreenCameraHasNoBenchmarkInlinePublication)
{
	EXPECT_FALSE((HasBenchmarkInlinePublicationHook<rendering::OffscreenCamera>::value));
}

TEST(LegacyProductionHooks, OffscreenCameraUsesPublicationCadenceNaming)
{
	EXPECT_FALSE((HasShouldRenderCadenceGate<rendering::OffscreenCamera>::value));
	EXPECT_TRUE((std::is_member_function_pointer_v<decltype(&rendering::OffscreenCamera::ShouldPublishAtTime)>));
}

TEST(LegacyProductionHooks, RenderingNamespaceHasNoCopyDataHelper)
{
	static_assert(!rendering::detail::kExportsCopyDataHelper,
	              "rendering::CopyData must stay removed; use SnapshotPool slots in WrappedStep");
}

#if OFFSCREEN_RENDER_BACKEND == EGL_BACKEND || OFFSCREEN_RENDER_BACKEND == OSMESA_BACKEND
class BlockingRenderBackend final : public rendering::IRenderBackend
{
public:
	rendering::RenderStatus Initialize(const mjModel &, const rendering::RenderConfiguration &) override
	{
		return rendering::RenderStatus::Ok();
	}

	rendering::RenderStatus Resize(const rendering::RenderConfiguration &) override
	{
		return rendering::RenderStatus::Ok();
	}

	rendering::RenderStatus Render(const rendering::RenderSnapshot &snapshot, const rendering::CameraDescriptor &camera,
	                               rendering::PlaneMask planes, rendering::FrameBoundary &boundary) override
	{
		std::unique_lock<std::mutex> lock(mutex_);
		rendered_model_generations_.push_back(snapshot.model_generation.value());
		rendered_snapshots_.push_back(&snapshot);
		rendered_data_.push_back(snapshot.data.get());
		rendered_plugin_geometry_.push_back(snapshot.plugin_geometry.get());
		rendered_camera_ids_.push_back(camera.id);
		render_entered_ = true;
		condition_.notify_all();
		const auto physics_mutex_probe = physics_mutex_probe_;
		if (physics_mutex_probe) {
			physics_mutex_available_.store(physics_mutex_probe());
		}
		if (auto_release_) {
			release_render_ = true;
		}
		condition_.wait(lock, [this] { return release_render_; });
		for (const auto plane :
		     { rendering::PlaneKind::kRgb, rendering::PlaneKind::kDepth, rendering::PlaneKind::kSegmentation }) {
			if (!rendering::HasPlane(planes, plane)) {
				continue;
			}
			auto writer = boundary.TryAcquireWriter(boundary.generation(), plane, camera.layout(plane));
			if (!writer.status().ok()) {
				return rendering::RenderStatus::Failure(rendering::RenderStatusCode::kFrameUnavailable,
				                                        writer.status().message);
			}
			if (!writer.Commit().ok()) {
				return rendering::RenderStatus::Failure(rendering::RenderStatusCode::kFrameUnavailable,
				                                        "frame commit failed");
			}
		}
		return render_status_;
	}

	void ShutdownOnRenderThread() override {}

	bool WaitUntilRenderEntered(std::chrono::seconds timeout)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return condition_.wait_for(lock, timeout, [this] { return render_entered_; });
	}

	void ReleaseRender()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			release_render_ = true;
		}
		condition_.notify_all();
	}

	void SetPhysicsMutexProbe(std::function<bool()> probe)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		physics_mutex_probe_ = std::move(probe);
	}

	void SetAutoRelease(bool enabled)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		auto_release_ = enabled;
	}

	bool PhysicsMutexAvailableDuringRender() const { return physics_mutex_available_.load(); }

	void PrepareNextRender(rendering::RenderStatus status)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		render_entered_ = false;
		release_render_ = false;
		render_status_  = std::move(status);
	}

	std::vector<std::uint64_t> RenderedModelGenerations()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return rendered_model_generations_;
	}
	std::vector<const rendering::RenderSnapshot *> RenderedSnapshots()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return rendered_snapshots_;
	}
	std::vector<const mjData *> RenderedData()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return rendered_data_;
	}
	std::vector<const std::vector<mjvGeom> *> RenderedPluginGeometry()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return rendered_plugin_geometry_;
	}
	std::vector<rendering::CameraId> RenderedCameraIds()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return rendered_camera_ids_;
	}

private:
	std::mutex mutex_;
	std::condition_variable condition_;
	std::function<bool()> physics_mutex_probe_;
	std::atomic_bool physics_mutex_available_{ false };
	bool render_entered_ = false;
	bool release_render_ = false;
	bool auto_release_   = false;
	rendering::RenderStatus render_status_;
	std::vector<std::uint64_t> rendered_model_generations_;
	std::vector<const rendering::RenderSnapshot *> rendered_snapshots_;
	std::vector<const mjData *> rendered_data_;
	std::vector<const std::vector<mjvGeom> *> rendered_plugin_geometry_;
	std::vector<rendering::CameraId> rendered_camera_ids_;
};

class BackendReleaseGuard
{
public:
	explicit BackendReleaseGuard(BlockingRenderBackend &backend) : backend_(backend) {}
	~BackendReleaseGuard() { backend_.ReleaseRender(); }

	BackendReleaseGuard(const BackendReleaseGuard &)            = delete;
	BackendReleaseGuard &operator=(const BackendReleaseGuard &) = delete;

private:
	BlockingRenderBackend &backend_;
};

class CallbackReleaseGuard
{
public:
	explicit CallbackReleaseGuard(std::function<void()> release) : release_(std::move(release)) {}
	~CallbackReleaseGuard()
	{
		if (release_) {
			release_();
		}
	}

	CallbackReleaseGuard(const CallbackReleaseGuard &)            = delete;
	CallbackReleaseGuard &operator=(const CallbackReleaseGuard &) = delete;

	void Release()
	{
		if (release_) {
			release_();
			release_ = {};
		}
	}

private:
	std::function<void()> release_;
};

struct ReloadObserverSync
{
	std::mutex mutex;
	std::condition_variable condition;
	bool entered = false;
	bool release = false;
};

class ReloadObserverReleaseGuard
{
public:
	explicit ReloadObserverReleaseGuard(ReloadObserverSync &sync) : sync_(sync) {}
	~ReloadObserverReleaseGuard() { Release(); }

	ReloadObserverReleaseGuard(const ReloadObserverReleaseGuard &)            = delete;
	ReloadObserverReleaseGuard &operator=(const ReloadObserverReleaseGuard &) = delete;

	void Release()
	{
		if (released_) {
			return;
		}
		{
			std::lock_guard<std::mutex> lock(sync_.mutex);
			sync_.release = true;
		}
		sync_.condition.notify_all();
		released_ = true;
	}

private:
	ReloadObserverSync &sync_;
	bool released_ = false;
};

class SelectiveRenderBackend final : public rendering::IRenderBackend
{
public:
	explicit SelectiveRenderBackend(const rendering::PlaneMask committed_planes) : committed_planes_(committed_planes) {}

	rendering::RenderStatus Initialize(const mjModel &, const rendering::RenderConfiguration &) override
	{
		return rendering::RenderStatus::Ok();
	}

	rendering::RenderStatus Resize(const rendering::RenderConfiguration &) override
	{
		return rendering::RenderStatus::Ok();
	}

	rendering::RenderStatus Render(const rendering::RenderSnapshot &, const rendering::CameraDescriptor &camera,
	                               const rendering::PlaneMask planes, rendering::FrameBoundary &boundary) override
	{
		for (const auto plane :
		     { rendering::PlaneKind::kRgb, rendering::PlaneKind::kDepth, rendering::PlaneKind::kSegmentation }) {
			if (!rendering::HasPlane(planes, plane) || !rendering::HasPlane(committed_planes_, plane)) {
				continue;
			}
			auto writer = boundary.TryAcquireWriter(boundary.generation(), plane, camera.layout(plane));
			if (!writer.status().ok()) {
				return rendering::RenderStatus::Failure(rendering::RenderStatusCode::kFrameUnavailable,
				                                        writer.status().message);
			}
			if (!writer.Commit().ok()) {
				return rendering::RenderStatus::Failure(rendering::RenderStatusCode::kFrameUnavailable,
				                                        "frame commit failed");
			}
		}
		return rendering::RenderStatus::Ok();
	}

	void ShutdownOnRenderThread() override {}

private:
	rendering::PlaneMask committed_planes_;
};

class RenderTeardownEnvWrapper : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;

	bool IsResetInProgressForTest() const { return reset_in_progress_.load(std::memory_order_acquire); }
	bool ReloadInProgressForTest() const { return reload_in_progress_.load(std::memory_order_acquire); }

	void RunBlockedRenderStep()
	{
		mujoco_ros::RecursiveLock physics_lock(physics_thread_mutex_);
		WrappedStep();
	}

	void DisableRegressionRenderConsumerForTest()
	{
		if (render_core_) {
			render_core_->SetConsumerEnabled(render_consumer_, false);
		}
	}

	void AssertRgbSubscriberReadyForTest(std::chrono::milliseconds timeout = std::chrono::seconds(2))
	{
		std::promise<void> connected;
		auto connected_future = connected.get_future();
		std::thread waiter([this, timeout, connected = std::move(connected)]() mutable {
			const auto deadline = Clock::now() + timeout;
			while (Clock::now() < deadline) {
				{
					std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
					if (!camera_publication_transport_.cams.empty() &&
					    camera_publication_transport_.cams[0]->rgb_pub_.getNumSubscribers() > 0) {
						connected.set_value();
						return;
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			connected.set_exception(
			    std::make_exception_ptr(std::runtime_error("RGB subscriber did not connect before deadline")));
		});
		ThreadJoinGuard join_guard(waiter);
		connected_future.get();
	}

	std::size_t InFlightRenderTurnCountForTest()
	{
		std::lock_guard<std::mutex> lock(render_turn_mutex_);
		return in_flight_render_turn_count_;
	}

	bool RenderTurnAdmissionOpenForTest()
	{
		std::lock_guard<std::mutex> lock(render_turn_mutex_);
		return render_turn_admission_open_;
	}

	void SetPhysicsPreLockProbe(std::function<void()> probe) { SetPhysicsPreLockTestHook(std::move(probe)); }

	void SetResetRejectedProbe(std::function<void()> probe) { SetResetRejectedTestHook(std::move(probe)); }

	void SetRetirementProbe(std::function<void()> probe) { SetRetirementTestHook(std::move(probe)); }

	std::size_t PythonRegistrationCountForTest()
	{
		std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
		return camera_publication_transport_.python_consumers.size();
	}

	std::size_t SnapshotPoolAllocationCountForTest() const { return snapshot_pool_.allocation_count(); }
	std::size_t SnapshotPoolCopyCountForTest() const { return snapshot_pool_.copy_count(); }
	std::size_t SnapshotPoolActiveLeaseCountForTest() const { return snapshot_pool_.active_lease_count(); }
	std::size_t CallbackSceneCountForTest() const { return RenderCallbackSceneCountForTest(); }
	void ResetBatchSnapshotAssemblyCountForTest() { MujocoEnv::ResetBatchSnapshotAssemblyCountForTest(); }
	std::size_t BatchSnapshotAssemblyCountForTest() const { return MujocoEnv::BatchSnapshotAssemblyCountForTest(); }
	rendering::SnapshotPool::AcquireResult AcquireSnapshotPoolLeaseForTest()
	{
		if (!model_ || !data_) {
			throw std::runtime_error("snapshot-pool test lease requires an active model");
		}
		return snapshot_pool_.Acquire(*model_, *data_, model_generation_);
	}

	bool WaitForPhysicsStoppedForTest(std::chrono::milliseconds timeout = std::chrono::seconds(2))
	{
		if (isPhysicsRunning() == 0) {
			return true;
		}
		const auto deadline = std::chrono::steady_clock::now() + timeout;
		while (isPhysicsRunning() != 0) {
			if (std::chrono::steady_clock::now() >= deadline) {
				return false;
			}
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
		return true;
	}

	void ShutdownRenderCoreForReloadFailureTest()
	{
		ASSERT_NE(render_core_, nullptr);
		render_core_->Shutdown();
	}

	std::string LoadErrorForTest() const { return { load_error_ }; }

	std::string PluginGenerationStatusForTest() const
	{
		try {
			const auto generation = plugin_host_->ActiveModelGeneration();
			return "active:" + std::to_string(generation.value());
		} catch (const std::exception &error) {
			return std::string("inactive: ") + error.what();
		} catch (...) {
			return "probe exception: unknown";
		}
	}

	bool HasActivePluginGenerationForTest() const { return PluginGenerationStatusForTest().rfind("active:", 0) == 0; }

	bool RenderActivationCoherentForTest()
	{
		std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
		if (!render_model_copy_ || !render_core_ || frame_generation_.value() == 0 ||
		    camera_publication_transport_.cams.empty()) {
			return false;
		}
		return std::all_of(camera_publication_transport_.cams.begin(), camera_publication_transport_.cams.end(),
		                   [&](const auto &camera) {
			                   return mujoco_ros::rendering::OffscreenCameraTestAccess::HasActiveRenderCore(*camera,
			                                                                                                render_core_);
		                   });
	}

	BlockingRenderBackend *InstallBlockingRenderCore()
	{
		if (render_core_) {
			render_core_->Shutdown();
		}
		auto backend      = std::make_unique<BlockingRenderBackend>();
		auto *backend_ptr = backend.get();
		render_core_      = std::make_unique<rendering::RenderCore>(std::move(backend));
		render_consumer_  = render_core_->RegisterContinuousConsumer("rendercore_completion_regression");

		std::vector<rendering::CameraDescriptor> cameras;
		for (int camera_id = 0; camera_id < model_->ncam; ++camera_id) {
			rendering::CameraDescriptor camera;
			camera.id        = static_cast<rendering::CameraId>(camera_id + 1);
			const char *name = mj_id2name(model_.get(), mjOBJ_CAMERA, camera_id);
			camera.name      = name != nullptr ? name : "camera_" + std::to_string(camera_id);
			camera.width     = std::max(1, model_->vis.global.offwidth);
			camera.height    = std::max(1, model_->vis.global.offheight);
			camera.planes    = rendering::PlaneMask::kRgb;
			cameras.push_back(std::move(camera));
		}
		const auto status = render_core_->Reconfigure(
		    model_generation_, frame_generation_,
		    rendering::FrameLayout(std::max(1, model_->vis.global.offwidth), std::max(1, model_->vis.global.offheight)),
		    cameras);
		if (!status.ok()) {
			throw std::runtime_error("could not install blocking RenderCore: " + status.message);
		}
		{
			std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
			frame_generation_ = render_core_->frames().generation();
			for (const auto &camera : camera_publication_transport_.cams) {
				camera->SetActiveRenderCore(render_core_);
				camera->RegisterConsumer(*render_core_);
			}
		}
		render_model_copy_ = rendering::CopyModel(*model_);
		return backend_ptr;
	}

	void InstallSelectiveRenderCore(const rendering::PlaneMask committed_planes)
	{
		if (render_core_) {
			render_core_->Shutdown();
		}
		auto backend     = std::make_unique<SelectiveRenderBackend>(committed_planes);
		render_core_     = std::make_unique<rendering::RenderCore>(std::move(backend));
		render_consumer_ = render_core_->RegisterContinuousConsumer("selective_render_test");

		std::vector<rendering::CameraDescriptor> cameras;
		cameras.reserve(camera_publication_transport_.cams.size());
		int max_width  = 1;
		int max_height = 1;
		for (const auto &camera : camera_publication_transport_.cams) {
			cameras.push_back(camera->descriptor());
			max_width  = std::max(max_width, cameras.back().width);
			max_height = std::max(max_height, cameras.back().height);
		}
		const auto status = render_core_->Reconfigure(model_generation_, frame_generation_,
		                                              rendering::FrameLayout(max_width, max_height), cameras);
		if (!status.ok()) {
			throw std::runtime_error("could not install selective RenderCore: " + status.message);
		}
		{
			std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
			frame_generation_ = render_core_->frames().generation();
			for (const auto &camera : camera_publication_transport_.cams) {
				camera->SetActiveRenderCore(render_core_);
				camera->RegisterConsumer(*render_core_);
			}
		}
		render_model_copy_ = rendering::CopyModel(*model_);
	}

	void RequestAuthoritativeShutdownForTest() { RequestShutdown(); }

	void RequestReloadForTest(const std::string &filename)
	{
		mju::strcpy_arr(queued_filename_, filename.c_str());
		requestLoad(2);
	}
};

TEST_F(BaseEnvFixture, PythonDemandDoesNotReplaceRosDemand)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	ASSERT_TRUE(offscreen->ActiveRenderCore());
	auto &camera = *offscreen->cams.front();

	const auto first_python_consumer  = camera.RegisterPythonConsumer(offscreen->ActiveRenderCore());
	const auto second_python_consumer = camera.RegisterPythonConsumer(offscreen->ActiveRenderCore());
	EXPECT_NE(first_python_consumer, second_python_consumer);
	EXPECT_TRUE(camera.ros_consumer_registered_);
	EXPECT_TRUE(camera.python_consumer_registered_);

	auto plan = offscreen->ActiveRenderCore()->EvaluateDemand(std::chrono::nanoseconds(0), camera.descriptor().id);
	EXPECT_EQ(std::find(plan.consumers.begin(), plan.consumers.end(), first_python_consumer), plan.consumers.end());
	EXPECT_EQ(std::find(plan.consumers.begin(), plan.consumers.end(), second_python_consumer), plan.consumers.end());

	offscreen->ActiveRenderCore()->RequestOneShot(first_python_consumer);
	offscreen->ActiveRenderCore()->RequestOneShot(second_python_consumer);
	plan = offscreen->ActiveRenderCore()->EvaluateDemand(std::chrono::nanoseconds(0), camera.descriptor().id);
	EXPECT_NE(std::find(plan.consumers.begin(), plan.consumers.end(), first_python_consumer), plan.consumers.end());
	EXPECT_NE(std::find(plan.consumers.begin(), plan.consumers.end(), second_python_consumer), plan.consumers.end());

	camera.UnregisterPythonConsumer(first_python_consumer);
	plan = offscreen->ActiveRenderCore()->EvaluateDemand(std::chrono::nanoseconds(0), camera.descriptor().id);
	EXPECT_EQ(std::find(plan.consumers.begin(), plan.consumers.end(), first_python_consumer), plan.consumers.end());
	EXPECT_NE(std::find(plan.consumers.begin(), plan.consumers.end(), second_python_consumer), plan.consumers.end());
	EXPECT_TRUE(camera.python_consumer_registered_);

	mjtNum simulation_time = 0.0;
	camera.UpdateDemand(*offscreen->ActiveRenderCore(), util::toRosTime(simulation_time));
	EXPECT_TRUE(camera.ros_consumer_registered_);
	EXPECT_TRUE(camera.python_consumer_registered_);

	camera.UnregisterPythonConsumer(second_python_consumer);
	EXPECT_TRUE(camera.ros_consumer_registered_);
	EXPECT_FALSE(camera.python_consumer_registered_);
}

TEST_F(BaseEnvFixture, IdlePythonRegistrationDoesNotForceRender)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 0.1);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the idle-python probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	const auto registration = offscreen->RegisterPythonConsumer(0);

	const auto renders_before_idle = backend->RenderedSnapshots().size();
	constexpr int idle_steps       = 4;
	for (int step = 0; step < idle_steps; ++step) {
		render_env->RunBlockedRenderStep();
	}
	EXPECT_EQ(backend->RenderedSnapshots().size(), renders_before_idle)
	    << "idle Python registration must not force offscreen renders";

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, AsyncPythonBufferCadenceFillsHistoryWithoutRead)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 0.1);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the async-buffer probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	const auto registration = offscreen->RegisterPythonConsumer(0, 2U);

	const auto renders_before_steps = backend->RenderedSnapshots().size();
	for (int step = 0; step < 3; ++step) {
		render_env->RunBlockedRenderStep();
	}
	EXPECT_GT(backend->RenderedSnapshots().size(), renders_before_steps)
	    << "async Python buffer must cadence-fill history without an explicit read";

	const auto lease = offscreen->AcquirePythonLatest(registration, rendering::PlaneKind::kRgb);
	ASSERT_TRUE(lease.has_value()) << "step-then-read must find a cadenced async frame";

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PythonBufferBindingStepThenBorrowProducesFrame)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 0.1);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the binding-path probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	const auto registration = python_buffer_test_access::RegisterBufferConsumer(*offscreen, 0, 2U);

	for (int step = 0; step < 3; ++step) {
		render_env->RunBlockedRenderStep();
	}
	const auto lease =
	    python_buffer_test_access::BorrowLatestThroughBuffer(*offscreen, registration, rendering::PlaneKind::kRgb);
	ASSERT_TRUE(lease.has_value()) << "public binding read path must succeed after step-then-borrow";
	EXPECT_GT(lease->capture_id(), 0U);
	EXPECT_EQ(backend->RenderedSnapshots().size(), 3U);

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, OnePythonReadRequestProducesOneCaptureWhenRosNotDue)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 0.1);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the one-shot python probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	const auto registration = offscreen->RegisterPythonConsumer(0);

	const auto renders_before_read = backend->RenderedSnapshots().size();
	for (int step = 0; step < 3; ++step) {
		render_env->RunBlockedRenderStep();
	}
	EXPECT_EQ(backend->RenderedSnapshots().size(), renders_before_read);

	(void)offscreen->AcquirePythonLatest(registration, rendering::PlaneKind::kRgb);
	render_env->RunBlockedRenderStep();
	EXPECT_EQ(backend->RenderedSnapshots().size(), renders_before_read + 1U)
	    << "one Python read request must produce one capture when ROS is not due";

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RosAndPythonReadShareOneCaptureWithSingleRender)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("shared_capture_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif

	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the shared-capture probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	render_env->AssertRgbSubscriberReadyForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	const auto registration = offscreen->RegisterPythonConsumer(0);
	(void)offscreen->AcquirePythonLatest(registration, rendering::PlaneKind::kRgb);

	const auto renders_before_shared_capture = backend->RenderedSnapshots().size();
	render_env->RunBlockedRenderStep();
	ASSERT_EQ(backend->RenderedSnapshots().size(), renders_before_shared_capture + 1U);

	const auto ros_capture_id = offscreen->cams[0]->last_published_capture_id();
	ASSERT_GT(ros_capture_id, 0U);
	const auto python_lease =
	    offscreen->ActiveRenderCore()->AcquireLatest(offscreen->cams[0]->descriptor().id, rendering::PlaneKind::kRgb);
	ASSERT_TRUE(python_lease.has_value());
	EXPECT_EQ(python_lease->capture_id(), ros_capture_id);

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PythonReadDemandDoesNotAdvanceRosPublicationCadence)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 10.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("python_cadence_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif

	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the python cadence probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	render_env->AssertRgbSubscriberReadyForTest();
	auto *offscreen         = render_env->getCameraPublicationTransport();
	auto &camera            = *offscreen->cams.front();
	const auto registration = offscreen->RegisterPythonConsumer(0);

	const auto renders_before = backend->RenderedSnapshots().size();
	render_env->RunBlockedRenderStep();
	ASSERT_EQ(backend->RenderedSnapshots().size(), renders_before + 1U);
	ASSERT_GT(camera.last_published_capture_id(), 0U);
	const auto ros_stamp_after_first_publish  = camera.last_pub_;
	const auto capture_id_after_first_publish = camera.last_published_capture_id();

	(void)offscreen->AcquirePythonLatest(registration, rendering::PlaneKind::kRgb);
	render_env->RunBlockedRenderStep();
	EXPECT_EQ(backend->RenderedSnapshots().size(), renders_before + 2U);
	EXPECT_EQ(camera.last_published_capture_id(), capture_id_after_first_publish)
	    << "Python-only demand must not publish another ROS frame";
	EXPECT_EQ(camera.last_pub_, ros_stamp_after_first_publish);

	offscreen->UnregisterPythonConsumer(registration);
	render_env->shutdown();
}

TEST(SimTimeNanosecondsConversion, AccumulatedMillisecondTimestepRoundsAtStep1007)
{
	mjtNum time = 0.0;
	for (int step = 0; step < 1007; ++step) {
		time += 0.001;
	}
	EXPECT_EQ(static_cast<std::int64_t>(time * 1e9), 1'006'999'999)
	    << "truncation must preserve the pre-rounded nanosecond value";
	EXPECT_EQ(util::simTimeToNanoseconds(time), 1'007'000'000)
	    << "rounded conversion must land on the exact millisecond boundary";
}

TEST(SimTimeNanosecondsConversion, RoundedTimestampsPassInclusiveCadenceGateFor5000Steps)
{
	constexpr std::int64_t kPeriodNanoseconds = 1'000'000;
	mjtNum time                               = 0.0;
	std::int64_t last_pub_ns                  = 0;
	std::size_t rejections                    = 0;
	for (std::size_t step = 0; step < 5000; ++step) {
		time += 0.001;
		const auto simulation_time_ns = util::simTimeToNanoseconds(time);
		if (simulation_time_ns - last_pub_ns < kPeriodNanoseconds) {
			++rejections;
			continue;
		}
		last_pub_ns = simulation_time_ns;
	}
	EXPECT_EQ(rejections, 0U) << "rounded simulation timestamps must not reject the inclusive cadence gate";
	EXPECT_EQ(last_pub_ns, 5'000'000'000);
}

TEST_F(BaseEnvFixture, OffscreenCameraAcceptsExactPublishPeriodBoundary)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 100.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);

	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &camera                              = *offscreen->cams.front();
	camera.pub_freq_                          = 100.f;
	constexpr std::int64_t kPeriodNanoseconds = 10'000'000;
	camera.last_pub_                          = util::toRosTime(0);
	camera.publication_sequence_.store(1, std::memory_order_release);

	EXPECT_FALSE(camera.ShouldPublishAtTime(util::toRosTime(kPeriodNanoseconds - 1)))
	    << "strictly before one publish period must not pass cadence gate";
	EXPECT_TRUE(camera.ShouldPublishAtTime(util::toRosTime(kPeriodNanoseconds)))
	    << "exactly one publish period elapsed must pass inclusive cadence gate";
	EXPECT_TRUE(camera.ShouldPublishAtTime(util::toRosTime(kPeriodNanoseconds + 1)))
	    << "after one publish period must pass cadence gate";

	render_env->shutdown();
}

TEST_F(BaseEnvFixture, TwoPythonRegistrationsShareOneSceneRender)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 0.1);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the dual-python probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->DisableRegressionRenderConsumerForTest();
	auto *offscreen                = render_env->getCameraPublicationTransport();
	const auto first_registration  = offscreen->RegisterPythonConsumer(0);
	const auto second_registration = offscreen->RegisterPythonConsumer(0);

	(void)offscreen->AcquirePythonLatest(first_registration, rendering::PlaneKind::kRgb);
	(void)offscreen->AcquirePythonLatest(second_registration, rendering::PlaneKind::kRgb);
	const auto renders_before = backend->RenderedSnapshots().size();
	render_env->RunBlockedRenderStep();
	ASSERT_EQ(backend->RenderedSnapshots().size(), renders_before + 1U);

	const auto first_lease =
	    offscreen->ActiveRenderCore()->AcquireLatest(offscreen->cams[0]->descriptor().id, rendering::PlaneKind::kRgb);
	const auto second_lease =
	    offscreen->ActiveRenderCore()->AcquireLatest(offscreen->cams[0]->descriptor().id, rendering::PlaneKind::kRgb);
	ASSERT_TRUE(first_lease.has_value());
	ASSERT_TRUE(second_lease.has_value());
	EXPECT_EQ(first_lease->capture_id(), second_lease->capture_id());

	offscreen->UnregisterPythonConsumer(first_registration);
	offscreen->UnregisterPythonConsumer(second_registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, NoDemandDoesNotAcquireSnapshotPoolSlot)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 1.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_EQ(render_env->SnapshotPoolAllocationCountForTest(), rendering::SnapshotPool::kCapacity);
	const auto copies_before          = render_env->SnapshotPoolCopyCountForTest();
	const auto callback_scenes_before = render_env->CallbackSceneCountForTest();

	ASSERT_TRUE(render_env->step(1));
	EXPECT_EQ(render_env->SnapshotPoolCopyCountForTest(), copies_before);
	EXPECT_EQ(render_env->CallbackSceneCountForTest(), callback_scenes_before);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, WrappedStepDoesNotAllocateOwnedDataPerCaptureAfterWarmup)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_EQ(render_env->SnapshotPoolAllocationCountForTest(), rendering::SnapshotPool::kCapacity);

	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the pool hot-path probe";

	const auto make_after_warmup   = rendering::SnapshotPool::make_data_operations();
	const auto delete_after_warmup = rendering::SnapshotPool::delete_data_operations();
	const auto copies_before       = render_env->SnapshotPoolCopyCountForTest();

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	constexpr int capture_count = 4;
	for (int capture = 0; capture < capture_count; ++capture) {
		render_env->RunBlockedRenderStep();
	}

	EXPECT_EQ(rendering::SnapshotPool::make_data_operations(), make_after_warmup)
	    << "production WrappedStep must reuse pooled mjData instead of calling mj_makeData per capture";
	EXPECT_EQ(rendering::SnapshotPool::delete_data_operations(), delete_after_warmup)
	    << "production WrappedStep must not delete pooled mjData per capture";
	EXPECT_EQ(render_env->SnapshotPoolCopyCountForTest(), copies_before + static_cast<std::size_t>(capture_count));

	render_env->shutdown();
}

TEST_F(BaseEnvFixture, SnapshotPoolExhaustionDropsWrappedStepWithoutWaiting)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the pool exhaustion probe";
	auto *backend = render_env->InstallBlockingRenderCore();
	// Pool capacity is two. Hold one slot externally; the blocked render step holds the second.
	auto held_first = render_env->AcquireSnapshotPoolLeaseForTest();
	ASSERT_TRUE(held_first.ok()) << held_first.message;

	std::promise<void> first_step_done;
	auto first_step_future = first_step_done.get_future();
	std::thread first_step([&] {
		render_env->RunBlockedRenderStep();
		first_step_done.set_value();
	});
	ThreadJoinGuard first_step_guard(first_step);
	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "blocked render step did not enter backend within timeout";
	EXPECT_EQ(render_env->SnapshotPoolActiveLeaseCountForTest(), rendering::SnapshotPool::kCapacity)
	    << "pool must be fully leased before the exhaustion probe";

	std::promise<void> dropped_step_done;
	auto dropped_step_future = dropped_step_done.get_future();
	std::thread dropped_step([&] {
		render_env->RunBlockedRenderStep();
		dropped_step_done.set_value();
	});
	ThreadJoinGuard dropped_step_guard(dropped_step);
	ASSERT_EQ(dropped_step_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "pool exhaustion blocked WrappedStep";
	EXPECT_EQ(render_env->GetRenderStatus().code, rendering::FrameStatusCode::kSnapshotPoolExhausted);

	backend->ReleaseRender();
	ASSERT_EQ(first_step_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "blocked render step did not finish after backend release";

	held_first.data.reset();
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, WrappedStepSharesOneOwnedBatchSnapshotAcrossFourCameras)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	for (int camera = 0; camera < 4; ++camera) {
		const std::string prefix = "cam_config/camera_" + std::to_string(camera);
		nh->setParam(prefix + "/stream_type", rendering::StreamType::RGB);
		nh->setParam(prefix + "/frequency", 2000.0);
	}

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
#if MJR_ROS_VERSION == ROS_2
	auto observer_node = std::make_shared<rclcpp::Node>("four_camera_shared_batch_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
#endif
	render_env->StartWithXML(testing::get_test_model_path("four_camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_EQ(render_env->getCameraPublicationTransport()->cams.size(), 4U);

#if MJR_ROS_VERSION == ROS_1
	std::vector<ros::Subscriber> subscribers;
	const boost::function<void(const sensor_msgs::Image::ConstPtr &)> no_op_callback =
	    [](const sensor_msgs::Image::ConstPtr &) {};
	for (int camera = 0; camera < 4; ++camera) {
		subscribers.push_back(nh->subscribe<sensor_msgs::Image>(
		    "cameras/camera_" + std::to_string(camera) + "/rgb/image_raw", 1, no_op_callback));
	}
#else
	std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers;
	for (int camera = 0; camera < 4; ++camera) {
		subscribers.push_back(observer_node->create_subscription<sensor_msgs::msg::Image>(
		    render_env->GetHandleNamespace() + "/cameras/camera_" + std::to_string(camera) + "/rgb/image_raw",
		    rclcpp::SensorDataQoS(), [](const sensor_msgs::msg::Image::ConstSharedPtr &) {}));
	}
#endif

	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the shared-batch probe";
	render_env->ResetBatchSnapshotAssemblyCountForTest();
	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetAutoRelease(true);
	render_env->RunBlockedRenderStep();

	EXPECT_EQ(render_env->BatchSnapshotAssemblyCountForTest(), 1U)
	    << "production WrappedStep must assemble one shared owned snapshot per capture batch";

	const auto snapshots       = backend->RenderedSnapshots();
	const auto data            = backend->RenderedData();
	const auto plugin_geometry = backend->RenderedPluginGeometry();
	const auto camera_ids      = backend->RenderedCameraIds();
	ASSERT_EQ(snapshots.size(), 4U);
	ASSERT_EQ(data.size(), 4U);
	ASSERT_EQ(plugin_geometry.size(), 4U);
	ASSERT_EQ(camera_ids.size(), 4U);
	for (std::size_t index = 1; index < 4; ++index) {
		EXPECT_EQ(snapshots[index], snapshots.front());
		EXPECT_EQ(data[index], data.front());
		EXPECT_EQ(plugin_geometry[index], plugin_geometry.front());
		EXPECT_NE(camera_ids[index], camera_ids.front());
	}
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RosAndPythonDeliverOneCaptureIdentity)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
	std::atomic<std::int64_t> last_image_stamp_ns{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images, &last_image_stamp_ns](const sensor_msgs::Image::ConstPtr &message) {
		    last_image_stamp_ns.store(static_cast<std::int64_t>(message->header.stamp.toNSec()));
		    received_images.fetch_add(1);
	    });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("capture_identity_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images, &last_image_stamp_ns](const sensor_msgs::msg::Image::ConstSharedPtr message) {
		    last_image_stamp_ns.store(static_cast<std::int64_t>(message->header.stamp.sec) * 1000000000LL +
		                              static_cast<std::int64_t>(message->header.stamp.nanosec));
		    received_images.fetch_add(1);
	    });
#endif

	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	ASSERT_TRUE(offscreen->ActiveRenderCore());
	const auto python_registration = offscreen->RegisterPythonConsumer(0);

	const auto deadline = Clock::now() + std::chrono::seconds(2);
	while (received_images.load() == 0 && Clock::now() < deadline) {
		render_env->step(1);
		std::this_thread::yield();
	}
	ASSERT_GT(received_images.load(), 0);
	const auto ros_capture_id = offscreen->cams[0]->last_published_capture_id();
	ASSERT_GT(ros_capture_id, 0U);
	const auto python_lease =
	    offscreen->ActiveRenderCore()->AcquireLatest(offscreen->cams[0]->descriptor().id, rendering::PlaneKind::kRgb);
	ASSERT_TRUE(python_lease.has_value());
	EXPECT_EQ(python_lease->capture_id(), ros_capture_id);
#if MJR_ROS_VERSION == ROS_1
	EXPECT_GT(last_image_stamp_ns.load(), 0);
	EXPECT_LE(last_image_stamp_ns.load(), static_cast<std::int64_t>(offscreen->cams[0]->last_pub_.toNSec()));
#else
	EXPECT_GT(last_image_stamp_ns.load(), 0);
	EXPECT_LE(last_image_stamp_ns.load(), offscreen->cams[0]->last_pub_.nanoseconds());
#endif

	offscreen->UnregisterPythonConsumer(python_registration);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RosPublishesAvailablePlaneAndReportsMissingConfiguredPlane)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_D);
	nh->setParam("cam_config/test_cam/frequency", 30.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_rgb{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_rgb](const sensor_msgs::Image::ConstPtr &) { received_rgb.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("partial_plane_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_rgb](const sensor_msgs::msg::Image::ConstSharedPtr) { received_rgb.fetch_add(1); });
#endif

	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	render_env->InstallSelectiveRenderCore(rendering::PlaneMask::kRgb);

	const auto deadline = Clock::now() + std::chrono::seconds(2);
	while (received_rgb.load() == 0 && Clock::now() < deadline) {
		render_env->step(1);
		std::this_thread::yield();
	}
	ASSERT_GT(received_rgb.load(), 0);
	const auto &status = offscreen->cams[0]->last_publication_status();
	EXPECT_EQ(status.code, rendering::FrameStatusCode::kFrameUnavailable);
	ASSERT_TRUE(status.plane.has_value());
	EXPECT_EQ(*status.plane, rendering::PlaneKind::kDepth);
	EXPECT_NE(status.message.find("depth"), std::string::npos);
	EXPECT_GT(offscreen->cams[0]->last_published_capture_id(), 0U);
	const auto render_status = render_env->GetRenderStatus();
	EXPECT_EQ(render_status.code, rendering::FrameStatusCode::kFrameUnavailable);
	EXPECT_NE(render_status.message.find("depth"), std::string::npos);

	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RenderBackpressureDoesNotHoldPhysicsMutex)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->RequestAuthoritativeShutdownForTest();
	ASSERT_TRUE(render_env->WaitForPhysicsStoppedForTest(std::chrono::seconds(2)))
	    << "background physics loop did not stop before the lock probe";

	auto *backend = render_env->InstallBlockingRenderCore();
	backend->SetPhysicsMutexProbe([&] {
		if (!render_env->getMutexPtr()->try_lock()) {
			return false;
		}
		render_env->getMutexPtr()->unlock();
		return true;
	});
	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2))) << "MujocoEnv did not submit the render turn";
	EXPECT_TRUE(backend->PhysicsMutexAvailableDuringRender())
	    << "physics mutex remained held while RenderCore backpressure blocked completion";

	backend->ReleaseRender();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "WrappedStep did not finish after RenderCore completed";
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PublicationWorkerAvoidsCallerMutexRendezvous)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->setParam("sim_steps", -1);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
#if MJR_ROS_VERSION == ROS_1
	const boost::function<void(const sensor_msgs::Image::ConstPtr &)> no_op_callback =
	    [](const sensor_msgs::Image::ConstPtr &) {};
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>("cameras/test_cam/rgb/image_raw", 1, no_op_callback);
#else
	auto observer_node = std::make_shared<rclcpp::Node>("publication_worker_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [](const sensor_msgs::msg::Image::ConstSharedPtr) {});
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->AssertRgbSubscriberReadyForTest();

	auto *backend   = render_env->InstallBlockingRenderCore();
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &blocked_camera = *offscreen->cams.front();

	std::mutex worker_barrier_mutex;
	std::condition_variable worker_barrier_cv;
	bool release_worker_hook = false;
	std::promise<void> worker_hook_entered;
	auto worker_hook_entered_future = worker_hook_entered.get_future();

	mujoco_ros::rendering::OffscreenCameraTestAccess::SetWorkerHook(blocked_camera, [&] {
		worker_hook_entered.set_value();
		std::unique_lock<std::mutex> lock(worker_barrier_mutex);
		worker_barrier_cv.wait(lock, [&] { return release_worker_hook; });
	});

	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);
	CallbackReleaseGuard worker_release_guard([&] {
		{
			std::lock_guard<std::mutex> lock(worker_barrier_mutex);
			release_worker_hook = true;
		}
		worker_barrier_cv.notify_all();
		mujoco_ros::rendering::OffscreenCameraTestAccess::ClearTestHooks(blocked_camera);
	});

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "owner WrappedStep did not enter blocked RenderCore";
	backend->ReleaseRender();

	ASSERT_EQ(worker_hook_entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "publication worker did not enter its hook";
	ASSERT_TRUE(mujoco_ros::rendering::OffscreenCameraTestAccess::TryLockPublicationMutex(blocked_camera))
	    << "publication worker blocked on publication_mutex_ while running queued hooks";
	mujoco_ros::rendering::OffscreenCameraTestAccess::UnlockPublicationMutex(blocked_camera);

	worker_release_guard.Release();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "accepted submission caller did not return after publication worker unblocked";
	mujoco_ros::rendering::OffscreenCameraTestAccess::WaitUntilPublicationWorkerIdle(blocked_camera);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PublishLatestResetDoesNotEnqueueStaleFrame)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->setParam("sim_steps", -1);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("reset_enqueue_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->AssertRgbSubscriberReadyForTest();
	received_images.store(0);

	auto *backend   = render_env->InstallBlockingRenderCore();
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &blocked_camera = *offscreen->cams.front();

	std::mutex enqueue_barrier_mutex;
	std::condition_variable enqueue_barrier_cv;
	bool release_enqueue_barrier = false;
	std::promise<void> enqueue_barrier_reached;
	const auto enqueue_barrier_reached_shared = enqueue_barrier_reached.get_future().share();

	mujoco_ros::rendering::OffscreenCameraTestAccess::SetEnqueueHook(blocked_camera, [&] {
		enqueue_barrier_reached.set_value();
		std::unique_lock<std::mutex> lock(enqueue_barrier_mutex);
		enqueue_barrier_cv.wait(lock, [&] { return release_enqueue_barrier; });
	});

	std::promise<void> reset_finished;
	auto reset_finished_future = reset_finished.get_future();
	std::thread reset_thread([&, enqueue_barrier_reached_shared]() {
		if (enqueue_barrier_reached_shared.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
			reset_finished.set_exception(
			    std::make_exception_ptr(std::runtime_error("enqueue barrier not reached before Reset()")));
			return;
		}
		blocked_camera.Reset();
		reset_finished.set_value();
	});
	ThreadJoinGuard reset_guard(reset_thread);

	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);
	CallbackReleaseGuard enqueue_release_guard([&] {
		{
			std::lock_guard<std::mutex> lock(enqueue_barrier_mutex);
			release_enqueue_barrier = true;
		}
		enqueue_barrier_cv.notify_all();
		mujoco_ros::rendering::OffscreenCameraTestAccess::ClearTestHooks(blocked_camera);
	});

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "owner WrappedStep did not enter blocked RenderCore";
	backend->ReleaseRender();

	ASSERT_EQ(enqueue_barrier_reached_shared.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "PublishLatest did not reach the enqueue barrier under publication_mutex_";
	EXPECT_EQ(reset_finished_future.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout)
	    << "Reset completed while PublishLatest still held publication_mutex_ at enqueue";

	enqueue_release_guard.Release();
	ASSERT_EQ(reset_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "Reset did not finish after PublishLatest released publication_mutex_";
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "accepted submission caller did not return after enqueue barrier release";

	mujoco_ros::rendering::OffscreenCameraTestAccess::WaitUntilPublicationWorkerIdle(blocked_camera);
	EXPECT_EQ(received_images.load(), 0) << "pre-reset frame was published after Reset() cancelled the queue";
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, ResetDuringWorkerPublishDoesNotDeliverStaleImage)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->setParam("sim_steps", -1);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("reset_worker_publish_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->AssertRgbSubscriberReadyForTest();

	auto *backend   = render_env->InstallBlockingRenderCore();
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &blocked_camera = *offscreen->cams.front();

	std::mutex worker_barrier_mutex;
	std::condition_variable worker_barrier_cv;
	bool release_worker_hook = false;
	std::promise<void> worker_hook_reached;
	const auto worker_hook_reached_shared = worker_hook_reached.get_future().share();

	mujoco_ros::rendering::OffscreenCameraTestAccess::SetWorkerHook(blocked_camera, [&] {
		worker_hook_reached.set_value();
		std::unique_lock<std::mutex> lock(worker_barrier_mutex);
		worker_barrier_cv.wait(lock, [&] { return release_worker_hook; });
	});

	std::promise<void> reset_finished;
	auto reset_finished_future = reset_finished.get_future();
	std::thread reset_thread([&, worker_hook_reached_shared]() {
		if (worker_hook_reached_shared.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
			reset_finished.set_exception(
			    std::make_exception_ptr(std::runtime_error("worker publish barrier not reached before Reset()")));
			return;
		}
		blocked_camera.Reset();
		reset_finished.set_value();
	});
	ThreadJoinGuard reset_guard(reset_thread);

	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);
	CallbackReleaseGuard worker_release_guard([&] {
		{
			std::lock_guard<std::mutex> lock(worker_barrier_mutex);
			release_worker_hook = true;
		}
		worker_barrier_cv.notify_all();
		mujoco_ros::rendering::OffscreenCameraTestAccess::ClearTestHooks(blocked_camera);
	});

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "owner WrappedStep did not enter blocked RenderCore";
	backend->ReleaseRender();

	ASSERT_EQ(worker_hook_reached_shared.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "publication worker did not reach the pre-publish hook barrier";
	ASSERT_EQ(reset_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "Reset did not finish while the worker remained blocked before ROS publish";

	worker_release_guard.Release();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "accepted submission caller did not return after stale worker publish was dropped";

	mujoco_ros::rendering::OffscreenCameraTestAccess::WaitUntilPublicationWorkerIdle(blocked_camera);
	EXPECT_EQ(received_images.load(), 0) << "pre-reset frame was published after Reset() advanced the sequence";
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, OffscreenCameraResetDoesNotRacePublication)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->setParam("sim_steps", -1);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("reset_publication_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &camera                   = *offscreen->cams.front();
	const auto subscriber_deadline = Clock::now() + std::chrono::seconds(2);
	while (camera.rgb_pub_.getNumSubscribers() == 0 && Clock::now() < subscriber_deadline) {
		std::this_thread::yield();
	}
	ASSERT_GT(camera.rgb_pub_.getNumSubscribers(), 0U);

	auto *backend = render_env->InstallBlockingRenderCore();
	offscreen     = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto &blocked_camera = *offscreen->cams.front();

	std::promise<void> reset_applied;
	auto reset_applied_future = reset_applied.get_future();
	mujoco_ros::rendering::OffscreenCameraTestAccess::SetHooks(blocked_camera, {}, {}, [&] {
		EXPECT_FALSE(render_env->isResetRequested()) << "reset request must be cleared before camera Reset() hooks run";
		EXPECT_TRUE(render_env->IsResetInProgressForTest()) << "physics must stay blocked until camera reset completes";
		reset_applied.set_value();
	});

	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "owner WrappedStep did not accept demand and enter RenderCore";

	render_env->requestReset();
	ASSERT_TRUE(render_env->isResetRequested());
	EXPECT_EQ(reset_applied_future.wait_for(std::chrono::milliseconds(500)), std::future_status::timeout)
	    << "owner reset completed while accepted pre-reset RenderCore work remained held";
	EXPECT_TRUE(render_env->isResetRequested())
	    << "owner reset request cleared while accepted pre-reset RenderCore work remained held";

	backend->ReleaseRender();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(5)), std::future_status::ready)
	    << "owner WrappedStep did not finish after RenderCore completed";
	ASSERT_EQ(reset_applied_future.wait_for(std::chrono::seconds(3)), std::future_status::ready)
	    << "owner EventLoop ProcessReset did not reset cameras after the old render drained";
	EXPECT_FALSE(render_env->isResetRequested());
	EXPECT_TRUE(blocked_camera.ShouldPublishAtTime(blocked_camera.last_pub_))
	    << "reset cadence was overwritten by in-flight publication";
	EXPECT_NE(blocked_camera.last_published_capture_id(), 0U)
	    << "accepted pre-reset capture identity was not committed before owner reset";
	mujoco_ros::rendering::OffscreenCameraTestAccess::SetHooks(blocked_camera, {}, {}, {});
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, ReloadWaitsForAcceptedSubmissionCompletionBeforeGenerationReplacement)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto image_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("reload_submission_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto image_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	const auto subscriber_deadline = Clock::now() + std::chrono::seconds(2);
	while (offscreen->cams[0]->rgb_pub_.getNumSubscribers() == 0 && Clock::now() < subscriber_deadline) {
		std::this_thread::yield();
	}
	ASSERT_GT(offscreen->cams[0]->rgb_pub_.getNumSubscribers(), 0U);

	auto *backend = render_env->InstallBlockingRenderCore();
	offscreen     = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	auto old_camera                 = offscreen->cams.front();
	const auto old_render_core      = offscreen->ActiveRenderCore();
	const auto old_frame_generation = offscreen->ActiveFrameGeneration();

	std::mutex publication_gap_mutex;
	std::condition_variable publication_gap_condition;
	bool publication_entered = false;
	bool release_publication = false;
	std::atomic_int event_order{ 0 };
	std::atomic_int publication_entered_order{ 0 };
	std::atomic_int step_finished_order{ 0 };
	std::atomic_int render_quiescence_started_order{ 0 };
	std::atomic_int old_generation_quiesced_order{ 0 };
	std::atomic_int new_generation_loaded_order{ 0 };
	std::atomic_bool physics_blocked_during_reconfigure{ false };
	std::atomic_bool render_activation_coherent{ false };
	std::atomic_bool old_camera_active_at_submission_idle{ false };
	std::atomic_uint64_t frame_generation_at_submission_idle{ std::numeric_limits<std::uint64_t>::max() };
	std::atomic_bool old_camera_active_at_quiescence{ true };
	std::atomic_uint64_t frame_generation_at_quiescence{ std::numeric_limits<std::uint64_t>::max() };
	std::mutex reload_phase_mutex;
	std::condition_variable reload_phase_condition;
	std::mutex admission_probe_mutex;
	std::condition_variable admission_probe_condition;
	bool admission_probe_entered = false;
	bool release_admission_probe = false;

	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderQuiescenceStarted) {
			render_quiescence_started_order.store(event_order.fetch_add(1) + 1);
		} else if (phase == MujocoEnv::ReloadPhase::kRenderTurnsIdle) {
			old_camera_active_at_submission_idle.store(
			    mujoco_ros::rendering::OffscreenCameraTestAccess::HasActiveRenderCore(*old_camera, old_render_core));
			{
				std::lock_guard<std::mutex> lock(offscreen->lifecycle_mutex);
				frame_generation_at_submission_idle.store(offscreen->ActiveFrameGenerationLocked().value());
			}
			std::unique_lock<std::mutex> lock(admission_probe_mutex);
			admission_probe_entered = true;
			admission_probe_condition.notify_all();
			admission_probe_condition.wait(lock, [&] { return release_admission_probe; });
		} else if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			std::promise<void> physics_probe_finished;
			auto physics_probe_future = physics_probe_finished.get_future();
			std::thread physics_probe([&] {
				render_env->RunBlockedRenderStep();
				physics_probe_finished.set_value();
			});
			ThreadJoinGuard physics_probe_guard(physics_probe);
			if (physics_probe_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
				ADD_FAILURE() << "physics admission remained open during RenderCore reconfiguration";
				return;
			}
			physics_blocked_during_reconfigure.store(render_env->InFlightRenderTurnCountForTest() == 0 &&
			                                         !render_env->RenderTurnAdmissionOpenForTest());
		} else if (phase == MujocoEnv::ReloadPhase::kOldGenerationQuiesced) {
			old_camera_active_at_quiescence.store(
			    mujoco_ros::rendering::OffscreenCameraTestAccess::HasActiveRenderCore(*old_camera, old_render_core));
			{
				std::lock_guard<std::mutex> lock(offscreen->lifecycle_mutex);
				frame_generation_at_quiescence.store(offscreen->ActiveFrameGenerationLocked().value());
			}
			old_generation_quiesced_order.store(event_order.fetch_add(1) + 1);
		} else if (phase == MujocoEnv::ReloadPhase::kNewGenerationLoaded) {
			render_activation_coherent.store(render_env->RenderActivationCoherentForTest());
			new_generation_loaded_order.store(event_order.fetch_add(1) + 1);
		}
		reload_phase_condition.notify_all();
	});
	mujoco_ros::rendering::OffscreenCameraTestAccess::SetHooks(
	    *old_camera,
	    [&] {
		    std::unique_lock<std::mutex> lock(publication_gap_mutex);
		    publication_entered_order.store(event_order.fetch_add(1) + 1);
		    publication_entered = true;
		    publication_gap_condition.notify_all();
		    publication_gap_condition.wait(lock, [&] { return release_publication; });
	    },
	    {}, {});

	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished_order.store(event_order.fetch_add(1) + 1);
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard backend_guard(*backend);
	CallbackReleaseGuard publication_guard([&] {
		{
			std::lock_guard<std::mutex> lock(publication_gap_mutex);
			release_publication = true;
		}
		publication_gap_condition.notify_all();
	});
	CallbackReleaseGuard admission_probe_guard([&] {
		{
			std::lock_guard<std::mutex> lock(admission_probe_mutex);
			release_admission_probe = true;
		}
		admission_probe_condition.notify_all();
	});

	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "owner WrappedStep did not enter blocked RenderCore";
	backend->ReleaseRender();
	ASSERT_TRUE([&] {
		std::unique_lock<std::mutex> lock(publication_gap_mutex);
		return publication_gap_condition.wait_for(lock, std::chrono::seconds(2), [&] { return publication_entered; });
	}()) << "accepted submission did not reach the RenderCore-to-PublishLatest gap";

	render_env->RequestReloadForTest(testing::get_test_model_path("camera_world.xml"));
	{
		std::unique_lock<std::mutex> lock(reload_phase_mutex);
		ASSERT_TRUE(reload_phase_condition.wait_for(lock, std::chrono::seconds(2), [&] {
			return render_quiescence_started_order.load() != 0;
		})) << "reload did not enter RenderCore quiescence while accepted publication remained held";
	}
	EXPECT_TRUE(mujoco_ros::rendering::OffscreenCameraTestAccess::HasActiveRenderCore(*old_camera, old_render_core))
	    << "old camera was deactivated while accepted publication remained held";
	{
		std::lock_guard<std::mutex> lock(offscreen->lifecycle_mutex);
		ASSERT_EQ(offscreen->cams.size(), 1U);
		ASSERT_EQ(offscreen->cams.front().get(), old_camera.get())
		    << "old camera was removed while accepted publication remained held";
		ASSERT_EQ(offscreen->ActiveRenderCore(), old_render_core)
		    << "old RenderCore association was cleared while accepted publication remained held";
		EXPECT_EQ(offscreen->ActiveFrameGenerationLocked(), old_frame_generation)
		    << "old frame generation changed while accepted publication remained held";
	}
	EXPECT_EQ(old_generation_quiesced_order.load(), 0)
	    << "reload reached old-generation quiescence before accepted publication completed";

	EXPECT_EQ(old_camera->last_published_capture_id(), 0U)
	    << "old camera published while its accepted publication was held";
	EXPECT_EQ(received_images.load(), 0) << "ROS consumer received a frame before accepted publication completed";

	{
		std::unique_lock<std::mutex> lock(reload_phase_mutex);
		EXPECT_EQ(reload_phase_condition.wait_for(lock, std::chrono::milliseconds(500),
		                                          [&] { return old_generation_quiesced_order.load() != 0; }),
		          false)
		    << "reload replaced generations while accepted submission caller remained in PublishLatest";
	}

	publication_guard.Release();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "accepted submission caller did not return after publication release";
	{
		std::unique_lock<std::mutex> lock(admission_probe_mutex);
		ASSERT_TRUE(admission_probe_condition.wait_for(lock, std::chrono::seconds(2), [&] {
			return admission_probe_entered;
		})) << "reload did not reach the accepted-submission idle boundary";
	}
	EXPECT_FALSE(render_env->RenderTurnAdmissionOpenForTest())
	    << "reload reopened render submission admission before retirement";
	std::promise<void> admission_probe_finished;
	auto admission_probe_future = admission_probe_finished.get_future();
	std::thread admission_probe_thread([&] {
		render_env->RunBlockedRenderStep();
		admission_probe_finished.set_value();
	});
	ASSERT_EQ(admission_probe_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "a physics step remained blocked after admission closed";
	admission_probe_thread.join();
	EXPECT_EQ(render_env->InFlightRenderTurnCountForTest(), 0U)
	    << "a post-idle physics step was admitted against the old render generation";
	EXPECT_TRUE(old_camera_active_at_submission_idle.load())
	    << "old camera was retired before the post-idle admission boundary completed";
	EXPECT_EQ(frame_generation_at_submission_idle.load(), old_frame_generation.value())
	    << "old frame generation changed before admission closure was observed";
	{
		std::lock_guard<std::mutex> lock(admission_probe_mutex);
		release_admission_probe = true;
	}
	admission_probe_condition.notify_all();
	{
		std::unique_lock<std::mutex> lock(reload_phase_mutex);
		ASSERT_TRUE(reload_phase_condition.wait_for(lock, std::chrono::seconds(2), [&] {
			return new_generation_loaded_order.load() != 0;
		})) << "reload did not complete new-generation activation";
	}
	ASSERT_TRUE(old_generation_quiesced_order.load() != 0)
	    << "reload did not report old-generation quiescence after submission completion";
	EXPECT_FALSE(old_camera_active_at_quiescence.load())
	    << "old camera remained active after accepted submission quiescence";
	EXPECT_EQ(frame_generation_at_quiescence.load(), 0U)
	    << "old frame generation remained active after accepted submission quiescence";
	EXPECT_LT(step_finished_order.load(), old_generation_quiesced_order.load())
	    << "reload reported old-generation quiescence before the submission caller returned";
	EXPECT_LT(old_generation_quiesced_order.load(), new_generation_loaded_order.load())
	    << "new generation activated before old generation quiescence";
	EXPECT_TRUE(physics_blocked_during_reconfigure.load())
	    << "physics admission was not closed during RenderCore reconfiguration";
	EXPECT_TRUE(render_activation_coherent.load())
	    << "new-generation observer saw partial render activation or an unpublished owned model";

	offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	EXPECT_NE(offscreen->cams.front().get(), old_camera.get())
	    << "reload did not replace the offscreen camera generation";
	const auto old_capture_id = old_camera->last_published_capture_id();
	EXPECT_GT(old_capture_id, 0U) << "accepted old-generation publication did not complete before retirement";

	backend->PrepareNextRender(rendering::RenderStatus::Ok());
	{
		std::thread new_step_thread([&] { render_env->RunBlockedRenderStep(); });
		ThreadJoinGuard new_step_guard(new_step_thread);
		ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
		    << "new-generation owner step did not enter RenderCore";
		backend->ReleaseRender();
	}
	EXPECT_EQ(old_camera->last_published_capture_id(), old_capture_id)
	    << "retired old camera published after new generation activation";
	EXPECT_EQ(backend->RenderedModelGenerations().size(), 2U);
	EXPECT_GT(backend->RenderedModelGenerations().back(), backend->RenderedModelGenerations().front())
	    << "new render did not use a newer model generation";

	mujoco_ros::rendering::OffscreenCameraTestAccess::SetHooks(*old_camera, {}, {}, {});
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, ReloadSuccessPublishesUsableStateAtNewGenerationLoaded)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	ReloadObserverSync observer_sync;
	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_TRUE(render_env->GetSimState().model_valid);
	const auto load_count_before = render_env->GetSimState().load_count;

	ReloadObserverReleaseGuard observer_release_guard(observer_sync);
	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase != MujocoEnv::ReloadPhase::kNewGenerationLoaded) {
			return;
		}
		std::unique_lock<std::mutex> lock(observer_sync.mutex);
		observer_sync.entered = true;
		observer_sync.condition.notify_all();
		observer_sync.condition.wait(lock, [&] { return observer_sync.release; });
	});

	render_env->RequestReloadForTest(model_path);
	{
		std::unique_lock<std::mutex> lock(observer_sync.mutex);
		ASSERT_TRUE(observer_sync.condition.wait_for(lock, std::chrono::seconds(2), [&] {
			return observer_sync.entered;
		})) << "reload did not reach kNewGenerationLoaded during generation replacement";
	}

	const auto control_snapshot = render_env->GetControlSnapshot();
	EXPECT_EQ(control_snapshot.load_request, 0) << "load_request must be cleared before kNewGenerationLoaded returns";
	EXPECT_EQ(control_snapshot.model_lifecycle, ModelLifecyclePhase::kOperational)
	    << "lifecycle must be operational before kNewGenerationLoaded returns";
	EXPECT_TRUE(render_env->GetSimState().model_valid) << "model_valid must be true before kNewGenerationLoaded returns";
	EXPECT_EQ(render_env->GetSimState().load_count, load_count_before + 1)
	    << "load_count must increment before kNewGenerationLoaded returns";
	EXPECT_FALSE(render_env->ReloadInProgressForTest())
	    << "reload_in_progress must be cleared before kNewGenerationLoaded returns";
	EXPECT_TRUE(render_env->RenderTurnAdmissionOpenForTest())
	    << "render admission must be open before kNewGenerationLoaded returns";

	observer_release_guard.Release();
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "reload did not complete after releasing the observer";
}

TEST_F(BaseEnvFixture, ReloadObserverThrowAfterPublicationStillCleansUpAndLeavesEventLoopAlive)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);

	std::atomic_bool observer_called{ false };
	std::atomic_bool manual_steps_admitted{ false };
	std::promise<void> failure_cleanup_finished;
	auto failure_cleanup_future = failure_cleanup_finished.get_future();
	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kNewGenerationLoaded) {
			observer_called.store(true);
			const auto control_snapshot = render_env->GetControlSnapshot();
			if (control_snapshot.load_request != 0 ||
			    control_snapshot.model_lifecycle != ModelLifecyclePhase::kOperational ||
			    !render_env->GetSimState().model_valid || render_env->ReloadInProgressForTest() ||
			    !render_env->RenderTurnAdmissionOpenForTest()) {
				throw std::runtime_error("post-publication observer saw incomplete success boundary");
			}
			if (!render_env->RequestManualSteps(2)) {
				throw std::runtime_error("manual steps were not admitted after successful reload publication");
			}
			if (render_env->GetControlSnapshot().pending_steps != 2) {
				throw std::runtime_error("admitted manual steps were not visible after successful reload publication");
			}
			manual_steps_admitted.store(true);
			throw std::runtime_error("deterministic post-publication reload observer failure");
		}
		if (phase == MujocoEnv::ReloadPhase::kReloadFailed) {
			failure_cleanup_finished.set_value();
		}
	});

	render_env->RequestReloadForTest(model_path);
	ASSERT_EQ(failure_cleanup_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "post-publication observer throw did not complete HandleReloadFailure cleanup";
	EXPECT_TRUE(observer_called.load());
	EXPECT_TRUE(manual_steps_admitted.load());
	EXPECT_TRUE(render_env->isEventRunning()) << "post-publication observer exception terminated the event loop";
	EXPECT_EQ(render_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_EQ(render_env->GetControlSnapshot().pending_steps, 0)
	    << "HandleReloadFailure must cancel manual steps admitted after publication";
	EXPECT_FALSE(render_env->HasActivePluginGenerationForTest());
	EXPECT_NE(render_env->PluginGenerationStatusForTest().find("inactive:"), std::string::npos)
	    << render_env->PluginGenerationStatusForTest();
	EXPECT_TRUE(render_env->RenderTurnAdmissionOpenForTest())
	    << "render admission did not reopen after post-publication failure cleanup";
	{
		std::lock_guard<std::mutex> lock(offscreen->lifecycle_mutex);
		EXPECT_TRUE(offscreen->cams.empty()) << "render resources were not retired after post-publication failure";
		EXPECT_EQ(offscreen->ActiveFrameGenerationLocked().value(), 0U);
	}

	render_env->requestShutdown();
	render_env->WaitForEventsJoin();
	EXPECT_FALSE(render_env->isEventRunning());
}

TEST_F(BaseEnvFixture, ReloadFailureLeavesExplicitNoModelStateAndEventLoopAlive)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_TRUE(render_env->GetControlSnapshot().model_lifecycle == ModelLifecyclePhase::kOperational);

	render_env->ShutdownRenderCoreForReloadFailureTest();
	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "reload failure left the event loop in a loading state";

	const auto control_snapshot = render_env->GetControlSnapshot();
	EXPECT_EQ(control_snapshot.model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_FALSE(render_env->LoadErrorForTest().empty());
	EXPECT_TRUE(render_env->isEventRunning()) << "reload failure terminated the event loop";
	EXPECT_EQ(render_env->GetNumCBReadyPlugins(), 0);
	EXPECT_FALSE(render_env->HasActivePluginGenerationForTest())
	    << "PluginHost retained an active generation after model/data cleanup";
	EXPECT_NE(render_env->PluginGenerationStatusForTest().find("inactive:"), std::string::npos)
	    << render_env->PluginGenerationStatusForTest();
	{
		std::lock_guard<std::mutex> lock(render_env->getCameraPublicationTransport()->lifecycle_mutex);
		EXPECT_TRUE(render_env->getCameraPublicationTransport()->cams.empty());
		EXPECT_EQ(render_env->getCameraPublicationTransport()->ActiveFrameGenerationLocked().value(), 0U);
	}
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, ReloadObserverThrowStillCleansUpAndLeavesEventLoopAlive)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	std::atomic_bool observer_called{ false };
	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			observer_called.store(true);
			throw std::runtime_error("deterministic reload observer failure");
		}
	});

	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "observer-throw reload left the event loop in a loading state";
	EXPECT_TRUE(observer_called.load());
	EXPECT_TRUE(render_env->isEventRunning()) << "observer exception terminated the event loop";
	EXPECT_EQ(render_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_EQ(render_env->GetNumCBReadyPlugins(), 0);
	EXPECT_FALSE(render_env->HasActivePluginGenerationForTest());
	EXPECT_NE(render_env->PluginGenerationStatusForTest().find("inactive:"), std::string::npos)
	    << render_env->PluginGenerationStatusForTest();

	render_env->requestShutdown();
	render_env->WaitForEventsJoin();
	EXPECT_FALSE(render_env->isEventRunning());
}

TEST_F(BaseEnvFixture, CameraRetirementFailureRetriesAndLeavesLoudFailureState)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	mujoco_ros::rendering::OffscreenCameraTestAccess::FailNextRetirement(*offscreen->cams.front());

	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "camera-retirement failure left reload in progress";
	EXPECT_EQ(render_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_NE(render_env->LoadErrorForTest().find("camera retirement"), std::string::npos)
	    << "camera retirement failure was not preserved in diagnostics";
	EXPECT_TRUE(render_env->RenderTurnAdmissionOpenForTest())
	    << "admission did not reopen after retry completed resource teardown";
	EXPECT_TRUE(render_env->isEventRunning());
	EXPECT_FALSE(render_env->HasActivePluginGenerationForTest());
	{
		std::lock_guard<std::mutex> lock(offscreen->lifecycle_mutex);
		EXPECT_TRUE(offscreen->cams.empty());
		EXPECT_EQ(offscreen->ActiveFrameGenerationLocked().value(), 0U);
	}
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PhysicsSkipsStaleStepAfterReloadFailureClearsModel)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", false);
	nh->setParam("unpause", false);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);

	std::promise<void> physics_pre_lock;
	auto physics_pre_lock_reached = physics_pre_lock.get_future();
	std::promise<void> release_physics;
	auto release_physics_wait = release_physics.get_future().share();
	std::atomic_bool reported{ false };
	render_env->SetPhysicsPreLockProbe([&] {
		bool expected = false;
		if (reported.compare_exchange_strong(expected, true)) {
			physics_pre_lock.set_value();
			release_physics_wait.wait();
		}
	});
	std::atomic_bool physics_released{ false };
	CallbackReleaseGuard physics_release_guard([&] {
		render_env->SetPhysicsPreLockProbe({});
		bool expected = false;
		if (physics_released.compare_exchange_strong(expected, true)) {
			release_physics.set_value();
		}
	});
	ASSERT_EQ(physics_pre_lock_reached.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "physics loop did not reach the pre-lock overlap barrier";

	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			throw std::runtime_error("deterministic physics-overlap reload failure");
		}
	});
	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)))
	    << "reload failure did not reach explicit no-model state";
	EXPECT_EQ(render_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_TRUE(render_env->isEventRunning());

	physics_release_guard.Release();
	render_env->requestShutdown();
	render_env->WaitForPhysicsJoin();
	render_env->WaitForEventsJoin();
	EXPECT_FALSE(render_env->isPhysicsRunning());
	EXPECT_FALSE(render_env->isEventRunning());
}

TEST_F(BaseEnvFixture, ResetAfterReloadFailureIsRejectedWithoutNullMuJoCoAccess)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", false);
	nh->setParam("unpause", false);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	render_env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			throw std::runtime_error("deterministic reset-after-failure");
		}
	});
	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));

	std::promise<void> reset_rejected;
	auto reset_rejected_seen = reset_rejected.get_future();
	render_env->SetResetRejectedProbe([&] { reset_rejected.set_value(); });
	render_env->requestReset();
	ASSERT_EQ(reset_rejected_seen.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "no-model reset was not explicitly rejected";
	EXPECT_FALSE(render_env->isResetRequested());
	EXPECT_NE(render_env->LoadErrorForTest().find("Reset rejected"), std::string::npos);
	EXPECT_FALSE(render_env->GetSimState().model_valid);
	EXPECT_TRUE(render_env->isEventRunning());
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RegisterPythonConsumerAcceptsCapHistoryAndRejectsAboveCap)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_NE(offscreen->ActiveRenderCore(), nullptr);
	ASSERT_FALSE(offscreen->cams.empty());

	std::uint64_t at_cap_registration = 0;
	ASSERT_NO_THROW(at_cap_registration = offscreen->RegisterPythonConsumer(0, rendering::kMaxPythonHistoryDepth));
	offscreen->UnregisterPythonConsumer(at_cap_registration);

	try {
		(void)offscreen->RegisterPythonConsumer(0, rendering::kMaxPythonHistoryDepth + 1);
		FAIL() << "expected history-depth rejection on active RenderCore";
	} catch (const std::runtime_error &error) {
		const std::string message = error.what();
		EXPECT_NE(message.find("history depth"), std::string::npos);
		EXPECT_NE(message.find("exceeds supported cap"), std::string::npos);
		EXPECT_EQ(message.find("RenderCore"), std::string::npos);
		EXPECT_EQ(message.find("Invalid camera"), std::string::npos);
	}

	render_env->shutdown();
}

TEST_F(BaseEnvFixture, PythonRegistrationIsRejectedDuringCameraRetirementAndReboundAfter)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env       = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	const auto model_path = testing::get_test_model_path("camera_world.xml");
	render_env->StartWithXML(model_path);
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	auto *offscreen               = render_env->getCameraPublicationTransport();
	std::uint64_t registration_id = 0;
	ASSERT_NO_THROW(registration_id = offscreen->RegisterPythonConsumer(0));
	const auto old_core       = offscreen->ActiveRenderCore();
	const auto old_generation = offscreen->ActiveFrameGeneration();
	ASSERT_NE(old_core, nullptr);
	std::optional<rendering::FrameLease> retained_lease;
	const auto initial_frame_deadline = Clock::now() + std::chrono::seconds(2);
	while (!retained_lease && Clock::now() < initial_frame_deadline) {
		(void)offscreen->AcquirePythonLatest(registration_id, rendering::PlaneKind::kRgb);
		render_env->step(1);
		retained_lease = old_core->AcquireLatest(offscreen->cams[0]->descriptor().id, rendering::PlaneKind::kRgb);
		std::this_thread::yield();
	}
	ASSERT_TRUE(retained_lease.has_value()) << "Python registration did not produce an initial frame";

	std::atomic_bool retirement_probe_called{ false };
	std::atomic_bool registration_rejected{ false };
	std::atomic_bool acquisition_rejected{ false };
	std::atomic_bool recent_acquisition_rejected{ false };
	std::atomic_bool retained_lease_readable_during_retirement{ false };
	render_env->SetRetirementProbe([&] {
		bool expected = false;
		if (!retirement_probe_called.compare_exchange_strong(expected, true)) {
			return;
		}
		retained_lease_readable_during_retirement.store(!retained_lease->bytes().empty());
		try {
			(void)offscreen->RegisterPythonConsumer(0);
		} catch (const std::exception &error) {
			registration_rejected.store(std::string(error.what()).find("retirement") != std::string::npos);
		}
		try {
			(void)offscreen->AcquirePythonLatest(registration_id, rendering::PlaneKind::kRgb);
		} catch (const std::exception &error) {
			acquisition_rejected.store(std::string(error.what()).find("acquisition") != std::string::npos);
		}
		try {
			(void)offscreen->AcquirePythonRecent(registration_id, rendering::PlaneKind::kRgb, 1);
		} catch (const std::exception &error) {
			recent_acquisition_rejected.store(std::string(error.what()).find("acquisition") != std::string::npos);
		}
	});
	render_env->RequestReloadForTest(model_path);
	ASSERT_TRUE(render_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));
	EXPECT_TRUE(retirement_probe_called.load());
	EXPECT_TRUE(registration_rejected.load())
	    << "Python registration was not rejected while the old camera was retiring";
	EXPECT_TRUE(acquisition_rejected.load())
	    << "Python frame acquisition was not rejected while the old camera was retiring";
	EXPECT_TRUE(recent_acquisition_rejected.load())
	    << "Python copied-frame acquisition was not rejected while the old camera was retiring";
	EXPECT_TRUE(retained_lease_readable_during_retirement.load())
	    << "an already-acquired FrameLease became unreadable during retirement";
	EXPECT_EQ(render_env->PythonRegistrationCountForTest(), 1U)
	    << "existing Python registration was lost instead of being rebound";
	EXPECT_NE(offscreen->ActiveFrameGeneration(), old_generation);
	std::optional<rendering::FrameLease> rebound_lease;
	const auto rebound_frame_deadline = Clock::now() + std::chrono::seconds(2);
	while (!rebound_lease && Clock::now() < rebound_frame_deadline) {
		try {
			(void)offscreen->AcquirePythonLatest(registration_id, rendering::PlaneKind::kRgb);
			render_env->step(1);
			rebound_lease = offscreen->ActiveRenderCore()->AcquireLatest(offscreen->cams[0]->descriptor().id,
			                                                             rendering::PlaneKind::kRgb);
		} catch (const std::exception &) {
		}
		std::this_thread::yield();
	}
	ASSERT_TRUE(rebound_lease.has_value()) << "rebound Python registration could not acquire a new-generation frame";
	EXPECT_EQ(rebound_lease->generation(), offscreen->ActiveFrameGeneration());
	render_env->SetRetirementProbe({});
	offscreen->UnregisterPythonConsumer(registration_id);
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, RenderCoreBlocksStepUntilSubscriberCompletionAcrossReload)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config");
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/frequency", 2000.0);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	std::atomic_int received_images{ 0 };
#if MJR_ROS_VERSION == ROS_1
	auto rgb_subscriber = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&received_images](const sensor_msgs::Image::ConstPtr &) { received_images.fetch_add(1); });
#else
	auto observer_node = std::make_shared<rclcpp::Node>("rendercore_completion_observer");
	render_env->AddNodeToExecutor(observer_node->get_node_base_interface());
	auto rgb_subscriber = observer_node->create_subscription<sensor_msgs::msg::Image>(
	    render_env->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw", rclcpp::SensorDataQoS(),
	    [&received_images](const sensor_msgs::msg::Image::ConstSharedPtr) { received_images.fetch_add(1); });
#endif
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_TRUE(render_env->isRenderingRunning());
	auto *offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	const auto subscriber_deadline = Clock::now() + std::chrono::seconds(2);
	while (offscreen->cams[0]->rgb_pub_.getNumSubscribers() == 0 && Clock::now() < subscriber_deadline) {
		std::this_thread::yield();
	}
	ASSERT_GT(offscreen->cams[0]->rgb_pub_.getNumSubscribers(), 0U);

	auto *backend = render_env->InstallBlockingRenderCore();
	std::promise<void> step_finished;
	auto step_finished_future = step_finished.get_future();
	std::thread step_thread([&] {
		render_env->RunBlockedRenderStep();
		step_finished.set_value();
	});
	ThreadJoinGuard step_guard(step_thread);
	BackendReleaseGuard release_guard(*backend);
	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2))) << "MujocoEnv did not submit the render turn";
	EXPECT_EQ(step_finished_future.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout)
	    << "WrappedStep finished before RenderCore completed";

	backend->ReleaseRender();
	ASSERT_EQ(step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "WrappedStep did not finish after RenderCore completed";
	const auto first_message_deadline = Clock::now() + std::chrono::seconds(2);
	while (received_images.load() == 0 && Clock::now() < first_message_deadline) {
		std::this_thread::yield();
	}
	EXPECT_GT(received_images.load(), 0) << "subscribed RGB consumer did not receive the completed render";
	const int images_before_reload = received_images.load();
	const auto first_generations   = backend->RenderedModelGenerations();
	ASSERT_EQ(first_generations.size(), 1U);

	render_env->load_filename(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_TRUE(render_env->isRenderingRunning());
	offscreen = render_env->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1U);
	const auto reload_subscriber_deadline = Clock::now() + std::chrono::seconds(2);
	while (offscreen->cams[0]->rgb_pub_.getNumSubscribers() == 0 && Clock::now() < reload_subscriber_deadline) {
		std::this_thread::yield();
	}
	ASSERT_GT(offscreen->cams[0]->rgb_pub_.getNumSubscribers(), 0U);

	backend->PrepareNextRender(
	    rendering::RenderStatus::Failure(rendering::RenderStatusCode::kBackendFailure, "injected post-reload failure"));
	std::promise<void> reload_step_finished;
	auto reload_step_finished_future = reload_step_finished.get_future();
	std::thread reload_step_thread([&] {
		render_env->RunBlockedRenderStep();
		reload_step_finished.set_value();
	});
	ThreadJoinGuard reload_step_guard(reload_step_thread);
	ASSERT_TRUE(backend->WaitUntilRenderEntered(std::chrono::seconds(2)))
	    << "reloaded MujocoEnv did not submit a new-generation render turn";
	EXPECT_EQ(reload_step_finished_future.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout)
	    << "post-reload WrappedStep finished before RenderCore completed";

	backend->ReleaseRender();
	ASSERT_EQ(reload_step_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "post-reload WrappedStep did not finish after the failed RenderCore turn";
	EXPECT_EQ(received_images.load(), images_before_reload);
	EXPECT_FALSE(render_env->GetRenderStatus().ok()) << "failed RenderCore status was not exposed";
	const auto all_generations = backend->RenderedModelGenerations();
	ASSERT_EQ(all_generations.size(), 2U);
	EXPECT_GT(all_generations.back(), all_generations.front())
	    << "reload did not advance the submitted model generation";
	render_env->shutdown();
}

TEST_F(BaseEnvFixture, OffscreenRenderShutdownChild)
{
	if (!run_render_teardown_child) {
		GTEST_SKIP() << "run only from the bounded parent process";
	}

	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	auto render_env = std::make_unique<RenderTeardownEnvWrapper>("", nh.get());
	render_env->StartWithXML(testing::get_test_model_path("camera_world.xml"));
	ASSERT_EQ(render_env->GetOperationalStatus(), 0);
	ASSERT_TRUE(render_env->isRenderingRunning());

	ASSERT_TRUE(render_env->step(1, false));
	render_env->RequestAuthoritativeShutdownForTest();
	EXPECT_TRUE(render_env->isShutdownRequested());
	EXPECT_TRUE(render_env->GetControlSnapshot().shutdown_requested);
	render_env->shutdown();
	EXPECT_EQ(render_env->isPhysicsRunning(), 0);
	EXPECT_EQ(render_env->isRenderingRunning(), 0);
	EXPECT_EQ(render_env->isEventRunning(), 0);
}

TEST_F(BaseEnvFixture, OffscreenRenderShutdownCompletesAllJoinsWithinDeadline)
{
	std::array<char, PATH_MAX> executable_buffer = {};
	const ssize_t executable_length = readlink("/proc/self/exe", executable_buffer.data(), executable_buffer.size());
	ASSERT_GT(executable_length, 0) << "could not resolve render test executable";
	ASSERT_LT(executable_length, static_cast<ssize_t>(executable_buffer.size()))
	    << "render test executable path is too long";
	const std::string executable(executable_buffer.data(), executable_length);

	std::array<char *, 4> child_args = {
		const_cast<char *>(executable.c_str()),
		const_cast<char *>("--render-teardown-child"),
		const_cast<char *>("--gtest_filter=BaseEnvFixture.OffscreenRenderShutdownChild"),
		nullptr,
	};
	pid_t child_pid = -1;
	ASSERT_EQ(posix_spawn(&child_pid, executable.c_str(), nullptr, nullptr, child_args.data(), environ), 0)
	    << "could not start bounded render teardown child";
	ChildProcessReaper reaper(child_pid);

	constexpr auto child_deadline = std::chrono::seconds(5);
	const auto deadline           = Clock::now() + child_deadline;
	int child_status              = 0;
	bool child_exited             = false;
	while (Clock::now() < deadline) {
		const pid_t wait_result = waitpid(child_pid, &child_status, WNOHANG);
		if (wait_result == child_pid) {
			child_exited = true;
			break;
		}
		ASSERT_NE(wait_result, -1) << "waitpid failed for render teardown child";
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	if (child_exited) {
		reaper.MarkReaped();
	}
	const bool child_timed_out = !child_exited;
	if (child_timed_out) {
		ASSERT_EQ(kill(child_pid, SIGKILL), 0) << "could not terminate hung render teardown child";
		while (waitpid(child_pid, &child_status, 0) == -1 && errno == EINTR) {
		}
		child_exited = true;
		reaper.MarkReaped();
	}

	EXPECT_FALSE(child_timed_out) << "render teardown child exceeded " << child_deadline.count()
	                              << " seconds and was terminated";
	ASSERT_TRUE(child_exited);
	EXPECT_TRUE(WIFEXITED(child_status)) << "render teardown child did not exit normally";
	if (WIFEXITED(child_status)) {
		EXPECT_EQ(WEXITSTATUS(child_status), 0) << "render teardown child reported a test failure";
	}
}

TEST_F(BaseEnvFixture, Not_Headless_Warn)
{
	nh->setParam("no_render", false);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->GetOperationalStatus(), 0) << "Model did not become operational before timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, NoRender_Params_Correct)
{
	nh->setParam("no_render", true);
	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

#if MJR_ROS_VERSION == ROS_1
	bool offscreen = true, headless = false;
	nh->getParam("render_offscreen", offscreen);
	nh->getParam("headless", headless);
	EXPECT_TRUE(headless);
	EXPECT_FALSE(offscreen);
#endif
	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_FALSE(env_ptr->settings_.render_offscreen);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Headless_params_correct)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.render_offscreen);
	EXPECT_TRUE(env_ptr->settings_.headless);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::RGB);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool img = false, info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::DEPTH);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::DEPTH);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool img = false, info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::SEGMENTED);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::SEGMENTED);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool img = false, info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			info = true;
		}
		if (img && info)
			break;
	}
	EXPECT_TRUE(img && info);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_DEPTH_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_D);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::RGB_D);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool found_rgb = false, found_depth = false;
	bool found_rgb_info = false, found_depth_info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		}
		if (found_rgb && found_depth && found_rgb_info && found_depth_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_rgb_info && found_depth_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_S);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::RGB_S);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool found_rgb = false, found_seg = false;
	bool found_rgb_info = false, found_seg_info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_seg && found_rgb_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_seg && found_rgb_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::DEPTH_S);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::DEPTH_S);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool found_depth = false, found_seg = false;
	bool found_depth_info = false, found_seg_info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_depth && found_seg && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_depth && found_seg && found_depth_info && found_seg_info);
	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_DEPTH_SEGMENTATION_Topics_Available)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_D_S);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_TRUE(offscreen->cams[0]->stream_type_ == rendering::StreamType::RGB_D_S);

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;
	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Default_Cam_Settings)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check default camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 15);
	EXPECT_EQ(offscreen->cams[0]->width_, 720);
	EXPECT_EQ(offscreen->cams[0]->height_, 480);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Resolution_Settings)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/width", 640);
	nh->setParam("cam_config/test_cam/height", 480);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 15);
	EXPECT_EQ(offscreen->cams[0]->width_, 640);
	EXPECT_EQ(offscreen->cams[0]->height_, 480);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Stream_BaseTopic_Relative)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_D_S);
	nh->setParam("cam_config/test_cam/topic", "alt_topic");

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_STREQ(offscreen->cams[0]->topic_.c_str(), "alt_topic");

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/depth/image_raw") {
			found_depth = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/alt_topic/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Stream_BaseTopic_Absolute)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB_D_S);
	nh->setParam("cam_config/test_cam/topic", "/alt_topic");

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_STREQ(offscreen->cams[0]->topic_.c_str(), "/alt_topic");

	bool found_rgb = false, found_depth = false, found_seg = false;
	bool found_rgb_info = false, found_depth_info = false, found_seg_info = false;

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	for (const auto &t : topics) {
		if (t.name == "/alt_topic/rgb/image_raw") {
			found_rgb = true;
		} else if (t.name == "/alt_topic/depth/image_raw") {
			found_depth = true;
		} else if (t.name == "/alt_topic/segmented/image_raw") {
			found_seg = true;
		} else if (t.name == "/alt_topic/rgb/camera_info") {
			found_rgb_info = true;
		} else if (t.name == "/alt_topic/depth/camera_info") {
			found_depth_info = true;
		} else if (t.name == "/alt_topic/segmented/camera_info") {
			found_seg_info = true;
		}
		if (found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info)
			break;
	}
	EXPECT_TRUE(found_rgb && found_depth && found_seg && found_rgb_info && found_depth_info && found_seg_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::RGB);
	nh->setParam("cam_config/test_cam/name_rgb", "alt_rgb");

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_rgb/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_rgb/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DEPTH_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::DEPTH);
	nh->setParam("cam_config/test_cam/name_depth", "alt_depth");

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_depth/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_depth/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, SEGMENT_Alternative_StreamName)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::SEGMENTED);
	nh->setParam("cam_config/test_cam/name_segment", "alt_seg");

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");

	bool img = false, found_info = false;

	auto topics = testing::get_available_topics_for_test(env_ptr.get());

	for (const auto &t : topics) {
		if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_seg/image_raw") {
			img = true;
		} else if (t.name == env_ptr->GetHandleNamespace() + "/cameras/test_cam/alt_seg/camera_info") {
			found_info = true;
		}
		if (img && found_info)
			break;
	}
	EXPECT_TRUE(img && found_info);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RGB_Published_Correctly)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/frequency", 30.);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

#if MJR_ROS_VERSION == ROS_1
	std::vector<sensor_msgs::Image> rgb_images;
	std::vector<sensor_msgs::CameraInfo> rgb_infos;

	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });
	ros::Subscriber info_sub = nh->subscribe<sensor_msgs::CameraInfo>(
	    "cameras/test_cam/rgb/camera_info", 1,
	    [&rgb_infos](const sensor_msgs::CameraInfo::ConstPtr &msg) { rgb_infos.emplace_back(*msg); });

	env_ptr->StartWithXML(xml_path);
	env_ptr->step(1);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();

	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->rgb_pub_.getNumSubscribers(), 1);

	// Wait for image to be published with 1s timeout
	float seconds = 0.f;
	while ((rgb_images.empty() || rgb_infos.empty()) && seconds < 1.f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, 1.f) << "RGB image not published within 1s";

	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	ASSERT_EQ(rgb_images.size(), 1);
	ASSERT_EQ(rgb_infos.size(), 1);

	ros::Time t1 = ros::Time::now();
	EXPECT_EQ(rgb_images[0].header.stamp, t1);
	EXPECT_STREQ(rgb_images[0].header.frame_id.c_str(), "test_cam_optical_frame");
	EXPECT_EQ(rgb_images[0].width, 7);
	EXPECT_EQ(rgb_images[0].height, 4);
	EXPECT_EQ(rgb_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	EXPECT_EQ(rgb_infos[0].header.stamp, t1);

	env_ptr->shutdown();
#else // MJR_ROS_VERSION == ROS_2
	env_ptr->StartWithXML(xml_path);
	env_ptr->step(1);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();

	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	env_ptr->shutdown();
#endif
}

TEST_F(BaseEnvFixture, Cam_Timing_Correct)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/frequency", 30.);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

#if MJR_ROS_VERSION == ROS_1
	std::vector<sensor_msgs::Image> rgb_images;
	std::vector<sensor_msgs::CameraInfo> rgb_infos;

	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });
	ros::Subscriber info_sub = nh->subscribe<sensor_msgs::CameraInfo>(
	    "cameras/test_cam/rgb/camera_info", 1,
	    [&rgb_infos](const sensor_msgs::CameraInfo::ConstPtr &msg) { rgb_infos.emplace_back(*msg); });

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check default camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	// Wait for image to be published with 400ms timeout
	float seconds = 0.f;
	while ((rgb_images.empty() || rgb_infos.empty()) && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_infos.size(), 1);
	ASSERT_EQ(rgb_images.size(), 1);

	ros::Time t1 = ros::Time::now();
	// Step the simulation to as to trigger the camera rendering
	mjModel *m  = env_ptr->getModelPtr();
	int n_steps = std::ceil((1.0 / 30.0) / env_ptr->getModelPtr()->opt.timestep);

	env_ptr->step(n_steps - 1);
	// wait a little
	std::this_thread::sleep_for(std::chrono::milliseconds(5));

	// should not have received a new image yet
	ASSERT_EQ(rgb_infos.size(), 1);
	ASSERT_EQ(rgb_images.size(), 1);

	env_ptr->step(1);
	// Wait for image to be published with 400ms timeout
	seconds = 0.f;
	while ((rgb_images.size() < 2 || rgb_infos.size() < 2) && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	// should now have received new image
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_infos.size(), 2);
	ASSERT_EQ(rgb_images.size(), 2);
	ros::Time t2 = ros::Time::now();

	ASSERT_EQ(rgb_images[0].header.stamp, t1);
	ASSERT_EQ(rgb_images[1].header.stamp, t2);

	ASSERT_EQ(rgb_infos[0].header.stamp, t1);
	ASSERT_EQ(rgb_infos[1].header.stamp, t2);

	// int n_steps = std::ceil((1.0 / 30.0) / env_ptr->getModelPtr()->opt.timestep);
	ros::Time t3 = t2 + ros::Duration((std::ceil((1.0 / 30.0) / m->opt.timestep)) * m->opt.timestep);
	// ros::Time t3 = t2 + (t2 - t1);
	// Step over next image trigger but before trigger after that
	env_ptr->step(2 * n_steps - 1);

	ASSERT_EQ(rgb_infos.size(), 3);
	ASSERT_EQ(rgb_images.size(), 3);

	// Check that the timestamps are as expected
	EXPECT_EQ(rgb_images[2].header.stamp, rgb_infos[2].header.stamp);
	EXPECT_EQ(rgb_images[2].header.stamp, t3);
	EXPECT_EQ(rgb_infos[2].header.stamp, t3);

	env_ptr->shutdown();
#else // MJR_ROS_VERSION == ROS_2
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);

	// Check default camera settings
	EXPECT_EQ(offscreen->cams[0]->cam_id_, 0);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->pub_freq_, 30);

	env_ptr->shutdown();
#endif
}

TEST_F(BaseEnvFixture, RGB_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

#if MJR_ROS_VERSION == ROS_1
	std::vector<sensor_msgs::Image> rgb_images;
	// Subscribe to topic
	ros::Subscriber rgb_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/rgb/image_raw", 1,
	    [&rgb_images](const sensor_msgs::Image::ConstPtr &msg) { rgb_images.emplace_back(*msg); });

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 400ms timeout
	float seconds = 0.f;
	while (rgb_images.empty() && seconds < .4f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .4f) << "RGB image not published within 400ms";

	ASSERT_EQ(rgb_images.size(), 1);
	EXPECT_EQ(rgb_images[0].data.size(), 7 * 4 * 3);
	EXPECT_EQ(rgb_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	env_ptr->shutdown();
#else // MJR_ROS_VERSION == ROS_2
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::RGB);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	env_ptr->shutdown();
#endif
}

TEST_F(BaseEnvFixture, DEPTH_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::DEPTH);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

#if MJR_ROS_VERSION == ROS_1
	std::vector<sensor_msgs::Image> depth_images;

	// Subscribe to topic
	ros::Subscriber depth_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/depth/image_raw", 1,
	    [&depth_images](const sensor_msgs::Image::ConstPtr &msg) { depth_images.emplace_back(*msg); });

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::DEPTH);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 200ms timeout
	float seconds = 0.f;
	while (depth_images.empty() && seconds < .2f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .2f) << "Depth image not published within 200ms";

	ASSERT_EQ(depth_images.size(), 1);
	EXPECT_EQ(depth_images[0].width, 7);
	EXPECT_EQ(depth_images[0].height, 4);
	EXPECT_EQ(depth_images[0].encoding, sensor_msgs::image_encodings::TYPE_32FC1);

	env_ptr->shutdown();
#else // MJR_ROS_VERSION == ROS_2
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::DEPTH);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	env_ptr->shutdown();
#endif
}

TEST_F(BaseEnvFixture, SEGMENTED_Image_Dtype)
{
	nh->setParam("no_render", false);
	nh->setParam("headless", true);
	nh->setParam("render_offscreen", true);
	nh->setParam("unpause", false);
	nh->deleteParam("cam_config"); // ensure no config from other tests is present
	nh->setParam("cam_config/test_cam/stream_type", rendering::StreamType::SEGMENTED);
	nh->setParam("cam_config/test_cam/width", 7);
	nh->setParam("cam_config/test_cam/height", 4);

	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

#if MJR_ROS_VERSION == ROS_1
	std::vector<sensor_msgs::Image> seg_images;

	// Subscribe to topic
	ros::Subscriber seg_sub = nh->subscribe<sensor_msgs::Image>(
	    "cameras/test_cam/segmented/image_raw", 1,
	    [&seg_images](const sensor_msgs::Image::ConstPtr &msg) { seg_images.emplace_back(*msg); });

	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::SEGMENTED);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	// Wait for image to be published with 200ms timeout
	float seconds = 0.f;
	while (seg_images.empty() && seconds < .2f) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001f;
	}
	EXPECT_LT(seconds, .2f) << "Segmentation image not published within 200ms";

	ASSERT_EQ(seg_images.size(), 1);
	EXPECT_EQ(seg_images[0].data.size(), 7 * 4 * 3);
	EXPECT_EQ(seg_images[0].encoding, sensor_msgs::image_encodings::RGB8);

	env_ptr->shutdown();
#else // MJR_ROS_VERSION == ROS_2
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_TRUE(env_ptr->settings_.render_offscreen);

	env_ptr->step(1);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	ASSERT_EQ(offscreen->cams.size(), 1);
	EXPECT_STREQ(offscreen->cams[0]->cam_name_.c_str(), "test_cam");
	EXPECT_EQ(offscreen->cams[0]->stream_type_, rendering::StreamType::SEGMENTED);
	EXPECT_EQ(offscreen->cams[0]->width_, 7);
	EXPECT_EQ(offscreen->cams[0]->height_, 4);

	env_ptr->shutdown();
#endif
}

#endif // OFFSCREEN_RENDER_BACKEND == EGL_BACKEND || OFFSCREEN_RENDER_BACKEND == OSMESA_BACKEND
       // any render backend available

#if OFFSCREEN_RENDER_BACKEND == NO_BACKEND // i.e. no offscreen render backend available
TEST_F(BaseEnvFixture, No_Render_Backend_Headless_Warn)
{
	nh->setParam("headless", true);
	std::string xml_path = testing::get_test_model_path("camera_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->GetOperationalStatus(), 0) << "Model did not become operational before timeout!";

	EXPECT_TRUE(env_ptr->settings_.headless);
	EXPECT_FALSE(env_ptr->settings_.render_offscreen);

	CameraPublicationTransport *offscreen = env_ptr->getCameraPublicationTransport();
	EXPECT_TRUE(offscreen->cams.empty());

	env_ptr->shutdown();
}
#endif // OFFSCREEN_RENDER_BACKEND == NO_BACKEND // i.e. no offscreen render backend available
