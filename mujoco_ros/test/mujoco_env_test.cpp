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

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <condition_variable>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <future>
#include <mutex>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/simulation_control_state.hpp>
#include <mujoco_ros/util.hpp>
#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/detail/viewer_connection_state.hpp>
#include <mujoco_ros/viewer.hpp>
#endif

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#endif

int main(int argc, char **argv)
{
#if MJR_ROS_VERSION == ROS_1
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_env_test");

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

class ControlStateTestWrapper : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
	using MujocoEnvTestWrapper::RecordCompletedManualStep;
	using MujocoEnvTestWrapper::RequestReload;
	using MujocoEnvTestWrapper::RequestViewerReset;
	using MujocoEnvTestWrapper::RequestViewerShutdown;
	using MujocoEnvTestWrapper::SetPaused;
	using MujocoEnvTestWrapper::SetViewerRealTimeIndex;

	template <typename Func>
	void PublishLoadRequestForTest(int load_request, Func &&publish_payload)
	{
		PublishLoadRequest(load_request, std::forward<Func>(publish_payload));
	}

	bool CanAcquireControlBoundaryForTest()
	{
		if (!control_state_boundary_mutex_.try_lock()) {
			return false;
		}
		control_state_boundary_mutex_.unlock();
		return true;
	}

	bool CanAcquirePhysicsBoundaryForTest()
	{
		if (!physics_thread_mutex_.try_lock()) {
			return false;
		}
		physics_thread_mutex_.unlock();
		return true;
	}

	ControlSpeedSnapshot ConsumeSpeedSettingsSnapshotForTest() { return ConsumeSpeedSettingsSnapshot(); }
};

class WarningTestWrapper : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::FrameSlotWarningCountForTesting;
	using MujocoEnvTestWrapper::LastFrameSlotWarningForTesting;
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
	using MujocoEnvTestWrapper::SetWarningClockForTesting;
	using MujocoEnvTestWrapper::WarnFrameSlotDrop;
};

#if RENDER_BACKEND == GLFW_BACKEND
TEST_F(BaseEnvFixture, ViewerRenderLoopExceptionalExitRejectsLaterLoads)
{
	if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
		GTEST_SKIP() << "native display unavailable";
	}

	env_ptr     = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	auto viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_ptr.get(), true, false);
	std::future<std::string> accepted_load;

	try {
		viewer->RenderLoop([&viewer, &accepted_load]() {
			accepted_load       = std::async(std::launch::async, [&viewer]() {
            try {
               viewer->Load(nullptr, nullptr, "accepted.xml", ModelGeneration(1));
               return std::string("load unexpectedly succeeded");
            } catch (const std::runtime_error &error) {
               return std::string(error.what());
            }
         });
			const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
			while (viewer->loadrequest.load() != 2 && std::chrono::steady_clock::now() < deadline) {
				std::this_thread::yield();
			}
			if (viewer->loadrequest.load() != 2) {
				throw std::runtime_error("load was not accepted before on-ready failure");
			}
			throw std::runtime_error("injected on-ready failure");
		});
		FAIL() << "RenderLoop did not preserve its original exception";
	} catch (const std::runtime_error &error) {
		EXPECT_STREQ(error.what(), "injected on-ready failure");
	}

	auto later_load = std::async(std::launch::async, [&viewer]() {
		try {
			viewer->Load(nullptr, nullptr, "unused.xml", ModelGeneration(1));
			return std::string("load unexpectedly succeeded");
		} catch (const std::runtime_error &error) {
			return std::string(error.what());
		}
	});

	const auto accepted_result = accepted_load.wait_for(std::chrono::milliseconds(500));
	const auto prompt_result   = later_load.wait_for(std::chrono::milliseconds(500));
	EXPECT_EQ(accepted_result, std::future_status::ready)
	    << "RenderLoop did not reject an accepted load promise on exceptional exit";
	EXPECT_EQ(prompt_result, std::future_status::ready) << "Load accepted a promise after exceptional RenderLoop exit";

	// Let the pre-fix implementation finish the blocked load so RED fails without hanging CTest.
	if (accepted_result != std::future_status::ready || prompt_result != std::future_status::ready) {
		viewer->exit_request.store(1);
		EXPECT_NO_THROW(viewer->RenderLoop());
	}
	ASSERT_EQ(accepted_load.wait_for(std::chrono::seconds(2)), std::future_status::ready);
	ASSERT_EQ(later_load.wait_for(std::chrono::seconds(2)), std::future_status::ready);
	EXPECT_EQ(accepted_load.get(), "viewer render loop is not accepting load requests");
	EXPECT_EQ(later_load.get(), "viewer render loop is not accepting load requests");

	viewer.reset();
	env_ptr->shutdown();
}

namespace {

constexpr int kScenarioSuccess                  = 0;
constexpr int kScenarioGlfwStartupFailed        = 1;
constexpr int kScenarioViewerNotConnected       = 10;
constexpr int kScenarioDestroyFinishedEarly     = 11;
constexpr int kScenarioDestroyTimedOut          = 12;
constexpr int kScenarioLateRegistrationAccepted = 16;
constexpr int kScenarioLifetimeFailed           = 17;
constexpr int kScenarioLateActivationRejected   = 18;
constexpr int kScenarioRenderThreadTimedOut     = 20;
constexpr int kScenarioLeaseDrainFinishedEarly  = 21;
constexpr int kScenarioChildExecFailed          = 127;
constexpr char kViolationMarkerFdEnv[]          = "MUJOCO_ROS_LIFETIME_VIOLATION_MARKER_FD";

struct RenderThreadReadiness
{
	const std::atomic_bool *viewer_constructed = nullptr;
	const std::atomic_bool *glfw_ready         = nullptr;
	const std::atomic_bool *fully_connected    = nullptr;
};

int ClassifyPreReadyRenderThreadFailure(const RenderThreadReadiness &readiness)
{
	if (readiness.viewer_constructed == nullptr || readiness.glfw_ready == nullptr ||
	    readiness.fully_connected == nullptr) {
		return kScenarioLifetimeFailed;
	}
	if (readiness.fully_connected->load(std::memory_order_acquire)) {
		return kScenarioLifetimeFailed;
	}
	if (!readiness.viewer_constructed->load(std::memory_order_acquire)) {
		return kScenarioGlfwStartupFailed;
	}
	if (!readiness.glfw_ready->load(std::memory_order_acquire)) {
		return kScenarioViewerNotConnected;
	}
	return kScenarioLifetimeFailed;
}

int ClassifyGlfwReadyWaitTimeout(const RenderThreadReadiness &readiness)
{
	if (readiness.viewer_constructed == nullptr || readiness.glfw_ready == nullptr ||
	    readiness.fully_connected == nullptr) {
		return kScenarioLifetimeFailed;
	}
	if (!readiness.viewer_constructed->load(std::memory_order_acquire)) {
		return kScenarioGlfwStartupFailed;
	}
	return kScenarioViewerNotConnected;
}

int ClassifyRenderThreadException(const RenderThreadReadiness &readiness)
{
	if (readiness.viewer_constructed == nullptr || readiness.glfw_ready == nullptr ||
	    readiness.fully_connected == nullptr) {
		return kScenarioLifetimeFailed;
	}
	try {
		throw;
	} catch (const std::runtime_error &) {
		if (readiness.fully_connected->load(std::memory_order_acquire)) {
			return kScenarioLifetimeFailed;
		}
		if (!readiness.viewer_constructed->load(std::memory_order_acquire)) {
			return kScenarioGlfwStartupFailed;
		}
		if (!readiness.glfw_ready->load(std::memory_order_acquire)) {
			return kScenarioViewerNotConnected;
		}
		return kScenarioLifetimeFailed;
	} catch (...) {
		return ClassifyPreReadyRenderThreadFailure(readiness);
	}
}

void MarkContractViolationCause()
{
	const char *fd_env = std::getenv(kViolationMarkerFdEnv);
	if (fd_env == nullptr || fd_env[0] == '\0') {
		return;
	}
	const int fd = std::atoi(fd_env);
	if (fd < 0) {
		return;
	}
	const char marker   = 1;
	const ssize_t bytes = ::write(fd, &marker, 1);
	(void)bytes;
}

class LifetimeViolationCausePipe
{
public:
	bool Create()
	{
		if (::pipe(pipe_fds_) != 0) {
			return false;
		}
		return true;
	}

	void ExportTo(std::vector<std::string> &entries) const
	{
		entries.emplace_back(std::string(kViolationMarkerFdEnv) + "=" + std::to_string(pipe_fds_[1]));
	}

	bool CauseWasMarked() const
	{
		if (pipe_fds_[0] < 0) {
			return false;
		}
		if (pipe_fds_[1] >= 0) {
			::close(pipe_fds_[1]);
		}
		char marker         = 0;
		const ssize_t bytes = ::read(pipe_fds_[0], &marker, 1);
		::close(pipe_fds_[0]);
		return bytes == 1 && marker == 1;
	}

	~LifetimeViolationCausePipe()
	{
		if (pipe_fds_[0] >= 0) {
			::close(pipe_fds_[0]);
		}
		if (pipe_fds_[1] >= 0) {
			::close(pipe_fds_[1]);
		}
	}

private:
	int pipe_fds_[2] = { -1, -1 };
};

bool IsExpectedLateActivationRuntimeError(const std::atomic_bool &teardown_started,
                                          const std::atomic_bool &fully_connected, const std::runtime_error &error)
{
	if (fully_connected.load(std::memory_order_acquire)) {
		return false;
	}
	if (!teardown_started.load(std::memory_order_acquire)) {
		return false;
	}
	const std::string_view message                                 = error.what();
	static constexpr std::array<const char *, 5> kExpectedMessages = {
		"viewer render loop activation rejected",
		"viewer render loop activation rejected: environment unavailable",
		"viewer connection rejected because environment is closing",
		"viewer connection canceled before environment became idle",
		"viewer connection rejected because viewer is not registered",
	};
	return std::any_of(kExpectedMessages.begin(), kExpectedMessages.end(),
	                   [message](const char *expected) { return message == expected; });
}

class LeaseTestEnv : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
	ConnectedViewersLease TakeLease() const { return AcquireConnectedViewersLease(); }
};

class HeadlessTestEnv : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
};

bool NativeDisplayIsAdvertised()
{
	return std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
}

std::string CurrentTestExecutablePath()
{
	char path[PATH_MAX] = {};
	if (::readlink("/proc/self/exe", path, sizeof(path) - 1) < 0) {
		return {};
	}
	return path;
}

template <typename Predicate>
bool WaitUntil(Predicate &&predicate, std::chrono::milliseconds timeout)
{
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (std::chrono::steady_clock::now() < deadline) {
		if (predicate()) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return predicate();
}

class RenderLoopExitMarker
{
public:
	explicit RenderLoopExitMarker(std::atomic_bool &render_loop_exited) : render_loop_exited_(render_loop_exited) {}
	~RenderLoopExitMarker() { render_loop_exited_.store(true, std::memory_order_release); }

private:
	std::atomic_bool &render_loop_exited_;
};

std::shared_ptr<ViewerConnectionState>
CopyConnectionForExit(const std::shared_ptr<ViewerConnectionState> &connection_for_exit, std::mutex *viewer_mutex)
{
	if (viewer_mutex != nullptr) {
		std::lock_guard<std::mutex> lock(*viewer_mutex);
		return connection_for_exit;
	}
	return connection_for_exit;
}

void RequestViewerExit(const std::shared_ptr<ViewerConnectionState> &connection_for_exit, std::mutex *viewer_mutex)
{
	if (auto connection = CopyConnectionForExit(connection_for_exit, viewer_mutex)) {
		connection->RequestViewerExit();
	}
}

bool JoinRenderThreadBounded(const std::shared_ptr<ViewerConnectionState> &connection_for_exit,
                             std::mutex *viewer_mutex, std::thread &render_thread,
                             const std::atomic_bool &render_loop_exited, std::chrono::milliseconds timeout)
{
	RequestViewerExit(connection_for_exit, viewer_mutex);
	const bool exited =
	    WaitUntil([&render_loop_exited]() { return render_loop_exited.load(std::memory_order_acquire); }, timeout);
	if (!exited) {
		return false;
	}
	if (render_thread.joinable()) {
		render_thread.join();
	}
	return true;
}

struct LifetimeChildEnvironment
{
	std::vector<std::string> entries;
	std::vector<char *> envp;

	explicit LifetimeChildEnvironment(const LifetimeViolationCausePipe *violation_pipe = nullptr)
	{
		constexpr const char kChildFlagPrefix[] = "MUJOCO_ROS_LIFETIME_CHILD=";
		constexpr const char kViolationPrefix[] = "MUJOCO_ROS_LIFETIME_VIOLATION_MARKER_FD=";
		for (char **var = environ; var != nullptr && *var != nullptr; ++var) {
			if (std::strncmp(*var, kChildFlagPrefix, sizeof(kChildFlagPrefix) - 1) != 0 &&
			    std::strncmp(*var, kViolationPrefix, sizeof(kViolationPrefix) - 1) != 0) {
				entries.emplace_back(*var);
			}
		}
		entries.emplace_back("MUJOCO_ROS_LIFETIME_CHILD=1");
		if (violation_pipe != nullptr) {
			violation_pipe->ExportTo(entries);
		}
		envp.reserve(entries.size() + 1);
		for (auto &entry : entries) {
			envp.push_back(entry.data());
		}
		envp.push_back(nullptr);
	}
};

class ScopedLifetimeParentEnvScrubber
{
public:
	ScopedLifetimeParentEnvScrubber()
	{
		if (const char *value = std::getenv("MUJOCO_ROS_LIFETIME_CHILD")) {
			previous_ = value;
			::unsetenv("MUJOCO_ROS_LIFETIME_CHILD");
		}
	}

	~ScopedLifetimeParentEnvScrubber()
	{
		if (previous_) {
			::setenv("MUJOCO_ROS_LIFETIME_CHILD", previous_->c_str(), 1);
		}
	}

private:
	std::optional<std::string> previous_;
};

void ExpectChildScenarioSuccess(const char *filter, int expected_result = kScenarioSuccess)
{
	ScopedLifetimeParentEnvScrubber scrub_parent_child_mode;
	const std::string executable = CurrentTestExecutablePath();
	ASSERT_FALSE(executable.empty()) << "failed to resolve current test executable path";

	std::string filter_arg           = std::string("--gtest_filter=") + filter;
	std::array<char *, 3> child_args = {
		const_cast<char *>(executable.c_str()),
		const_cast<char *>(filter_arg.c_str()),
		nullptr,
	};

	LifetimeChildEnvironment child_env;
	pid_t child = -1;
	ASSERT_EQ(posix_spawn(&child, executable.c_str(), nullptr, nullptr, child_args.data(), child_env.envp.data()), 0)
	    << "posix_spawn failed for child scenario";

	int status               = 0;
	const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
	while (std::chrono::steady_clock::now() < wait_deadline) {
		const pid_t waited = ::waitpid(child, &status, WNOHANG);
		if (waited == child) {
			if (WIFEXITED(status) && WEXITSTATUS(status) == kScenarioGlfwStartupFailed) {
				GTEST_SKIP() << "GLFW startup failed in child (display unavailable)";
			}
			ASSERT_TRUE(WIFEXITED(status)) << "child did not exit cleanly";
			EXPECT_EQ(WEXITSTATUS(status), expected_result) << "child scenario failed with code " << WEXITSTATUS(status);
			return;
		}
		if (waited < 0) {
			FAIL() << "waitpid failed";
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	::kill(child, SIGKILL);
	::waitpid(child, &status, 0);
	FAIL() << "child process timed out";
}

void ExpectChildScenarioAborts(const char *filter)
{
	ScopedLifetimeParentEnvScrubber scrub_parent_child_mode;
	LifetimeViolationCausePipe violation_pipe;
	ASSERT_TRUE(violation_pipe.Create()) << "failed to create contract-violation marker pipe";

	const std::string executable = CurrentTestExecutablePath();
	ASSERT_FALSE(executable.empty()) << "failed to resolve current test executable path";

	std::string filter_arg           = std::string("--gtest_filter=") + filter;
	std::array<char *, 3> child_args = {
		const_cast<char *>(executable.c_str()),
		const_cast<char *>(filter_arg.c_str()),
		nullptr,
	};

	LifetimeChildEnvironment child_env(&violation_pipe);
	pid_t child = -1;
	ASSERT_EQ(posix_spawn(&child, executable.c_str(), nullptr, nullptr, child_args.data(), child_env.envp.data()), 0)
	    << "posix_spawn failed for child scenario";

	int status               = 0;
	const auto wait_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
	while (std::chrono::steady_clock::now() < wait_deadline) {
		const pid_t waited = ::waitpid(child, &status, WNOHANG);
		if (waited == child) {
			if (WIFEXITED(status) && WEXITSTATUS(status) == kScenarioGlfwStartupFailed) {
				GTEST_SKIP() << "GLFW startup failed in child (display unavailable)";
			}
			if (WIFEXITED(status) && WEXITSTATUS(status) == kScenarioViewerNotConnected) {
				FAIL() << "child reported viewer startup failure instead of expected contract violation";
			}
			if (WIFEXITED(status)) {
				FAIL() << "child exited with code " << WEXITSTATUS(status) << " instead of expected abort";
			}
			ASSERT_TRUE(WIFSIGNALED(status)) << "child did not abort as expected";
			const int signal = WTERMSIG(status);
			if (signal != SIGABRT) {
				FAIL() << "child terminated with unexpected signal " << signal << " (expected SIGABRT contract violation)";
			}
			EXPECT_TRUE(violation_pipe.CauseWasMarked())
			    << "child aborted without publishing the contract-violation cause marker";
			EXPECT_EQ(signal, SIGABRT);
			return;
		}
		if (waited < 0) {
			FAIL() << "waitpid failed";
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	::kill(child, SIGKILL);
	::waitpid(child, &status, 0);
	FAIL() << "child process timed out";
}

int RunConnectedViewerLeaseReleaseScenario(testing::TestNodeHandle *nh)
{
	auto env                    = std::make_unique<LeaseTestEnv>("", nh);
	LeaseTestEnv *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::optional<ConnectedViewersLease> lease;
	std::atomic_bool destroy_finished{ false };
	std::atomic_bool teardown_started{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &viewer_connected,
	                           &glfw_ready, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
			render_loop_exited.store(true, std::memory_order_release);
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}

	lease.emplace(env_raw->TakeLease());
	std::thread destroy_thread([&env, &destroy_finished, &teardown_started]() {
		teardown_started.store(true, std::memory_order_release);
		env.reset();
		destroy_finished.store(true, std::memory_order_release);
	});

	if (WaitUntil([&destroy_finished]() { return destroy_finished.load(std::memory_order_acquire); },
	              std::chrono::milliseconds(300))) {
		if (!render_loop_exited.load(std::memory_order_acquire)) {
			lease.reset();
			if (destroy_thread.joinable()) {
				destroy_thread.join();
			}
			cleanup_render_thread();
			return kScenarioLeaseDrainFinishedEarly;
		}
		lease.reset();
		if (destroy_thread.joinable()) {
			destroy_thread.join();
		}
		if (render_thread.joinable()) {
			render_thread.join();
		}
		return kScenarioSuccess;
	}

	lease.reset();
	if (destroy_thread.joinable()) {
		destroy_thread.join();
	}
	if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
	                             std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	if (!destroy_finished.load(std::memory_order_acquire)) {
		return kScenarioDestroyTimedOut;
	}
	return kScenarioSuccess;
}

int RunExternallyOwnedViewerSurvivesScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool render_finished{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &viewer_connected,
	                           &glfw_ready, &render_finished, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
			if (local_viewer->exit_request.load() != 2) {
				throw std::runtime_error("viewer did not disconnect cleanly after environment teardown");
			}
			render_finished.store(true, std::memory_order_release);
		} catch (const std::runtime_error &error) {
			if (viewer_connected.load(std::memory_order_acquire)) {
				render_result.store(kScenarioLifetimeFailed, std::memory_order_release);
			} else {
				render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
				                    std::memory_order_release);
				(void)error;
			}
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}

	env.reset();

	if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
	                             std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	if (!render_finished.load(std::memory_order_acquire)) {
		return kScenarioLifetimeFailed;
	}
	return kScenarioSuccess;
}

int RunViewerDestructorDuringRenderLoopScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::shared_ptr<Viewer> viewer_owner;
	std::mutex viewer_mutex;
	std::condition_variable viewer_cv;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool held_in_on_ready{ false };
	std::atomic_bool release_on_ready{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([&]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				viewer_owner = std::make_shared<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
				viewer_constructed.store(true, std::memory_order_release);
			}
			viewer_cv.notify_all();
			viewer_owner->RenderLoop([&glfw_ready, &held_in_on_ready, &release_on_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				held_in_on_ready.store(true, std::memory_order_release);
				WaitUntil([&release_on_ready]() { return release_on_ready.load(std::memory_order_acquire); },
				          std::chrono::seconds(5));
			});
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &held_in_on_ready }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		std::shared_ptr<ViewerConnectionState> connection;
		{
			std::lock_guard<std::mutex> lock(viewer_mutex);
			connection = viewer_owner ? viewer_owner->connection_state_ : nullptr;
		}
		JoinRenderThreadBounded(connection, &viewer_mutex, render_thread, render_loop_exited, std::chrono::seconds(10));
	};

	{
		std::unique_lock<std::mutex> lock(viewer_mutex);
		if (!viewer_cv.wait_for(lock, std::chrono::seconds(5), [&viewer_owner]() { return viewer_owner != nullptr; })) {
			release_on_ready.store(true, std::memory_order_release);
			cleanup_render_thread();
			return kScenarioGlfwStartupFailed;
		}
	}
	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		release_on_ready.store(true, std::memory_order_release);
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &held_in_on_ready });
	}
	if (!WaitUntil([&held_in_on_ready]() { return held_in_on_ready.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		release_on_ready.store(true, std::memory_order_release);
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		release_on_ready.store(true, std::memory_order_release);
		cleanup_render_thread();
		return render_result.load(std::memory_order_acquire);
	}

	std::shared_ptr<Viewer> doomed;
	{
		std::lock_guard<std::mutex> lock(viewer_mutex);
		doomed = std::move(viewer_owner);
	}
	MarkContractViolationCause();
	doomed.reset();
	std::terminate();
}

int RunEnvironmentDestructorFromViewerThreadScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([&env, env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &glfw_ready,
	                           &viewer_connected, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&env, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				MarkContractViolationCause();
				env.reset();
			});
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	cleanup_render_thread();
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	return kScenarioSuccess;
}

int RunLeaseBlocksEnvironmentModelResetScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::shared_ptr<Viewer> viewer;
	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool held_in_on_ready{ false };
	std::atomic_bool release_on_ready{ false };
	std::atomic_bool destroy_finished{ false };
	std::atomic_bool teardown_started{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([&]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			viewer = std::make_shared<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			viewer->RenderLoop([&viewer_connected, &glfw_ready, &held_in_on_ready, &release_on_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
				held_in_on_ready.store(true, std::memory_order_release);
				WaitUntil([&release_on_ready]() { return release_on_ready.load(std::memory_order_acquire); },
				          std::chrono::seconds(10));
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
			viewer.reset();
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		release_on_ready.store(true, std::memory_order_release);
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}
	if (!WaitUntil([&held_in_on_ready]() { return held_in_on_ready.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}

	const auto observe_connection_for_exit = [&]() {
		std::shared_ptr<ViewerConnectionState> connection_observed;
		{
			std::lock_guard<std::mutex> lock(viewer_mutex);
			connection_observed = connection_for_exit;
		}
		return connection_observed;
	};

	if (const auto connection_observed = observe_connection_for_exit();
	    !connection_observed || !connection_observed->RenderLoopActive()) {
		cleanup_render_thread();
		return kScenarioLifetimeFailed;
	}

	std::thread destroy_thread([&env, &destroy_finished, &teardown_started]() {
		teardown_started.store(true, std::memory_order_release);
		env.reset();
		destroy_finished.store(true, std::memory_order_release);
	});

	if (WaitUntil([&destroy_finished]() { return destroy_finished.load(std::memory_order_acquire); },
	              std::chrono::milliseconds(300))) {
		if (const auto connection_observed = observe_connection_for_exit();
		    connection_observed && connection_observed->RenderLoopActive()) {
			release_on_ready.store(true, std::memory_order_release);
			if (destroy_thread.joinable()) {
				destroy_thread.join();
			}
			cleanup_render_thread();
			return kScenarioDestroyFinishedEarly;
		}
		release_on_ready.store(true, std::memory_order_release);
		if (destroy_thread.joinable()) {
			destroy_thread.join();
		}
		if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                             std::chrono::seconds(10))) {
			if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
				return render_result.load(std::memory_order_acquire);
			}
			return kScenarioRenderThreadTimedOut;
		}
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioSuccess;
	}

	if (const auto connection_observed = observe_connection_for_exit();
	    !connection_observed || !connection_observed->RenderLoopActive()) {
		release_on_ready.store(true, std::memory_order_release);
		if (destroy_thread.joinable()) {
			destroy_thread.join();
		}
		cleanup_render_thread();
		return kScenarioLifetimeFailed;
	}

	release_on_ready.store(true, std::memory_order_release);
	if (destroy_thread.joinable()) {
		destroy_thread.join();
	}
	if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
	                             std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	if (!destroy_finished.load(std::memory_order_acquire)) {
		return kScenarioDestroyTimedOut;
	}
	return kScenarioSuccess;
}

int RunLateViewerActivationRejectedScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();
	env_raw->requestLoad(1);

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool viewer_registered{ false };
	std::atomic_bool viewer_fully_connected{ false };
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool teardown_started{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_registered, &viewer_fully_connected,
	                           &glfw_ready, &teardown_started, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_registered.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_fully_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_fully_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
		} catch (const std::runtime_error &error) {
			if (viewer_fully_connected.load(std::memory_order_acquire)) {
				render_result.store(kScenarioLifetimeFailed, std::memory_order_release);
			} else if (!IsExpectedLateActivationRuntimeError(teardown_started, viewer_fully_connected, error)) {
				render_result.store(
				    ClassifyPreReadyRenderThreadFailure({ &viewer_registered, &glfw_ready, &viewer_fully_connected }),
				    std::memory_order_release);
			}
		} catch (...) {
			render_result.store(
			    ClassifyRenderThreadException({ &viewer_registered, &glfw_ready, &viewer_fully_connected }),
			    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&viewer_registered]() { return viewer_registered.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioGlfwStartupFailed;
	}

	std::thread destroy_thread([&env, &teardown_started]() {
		teardown_started.store(true, std::memory_order_release);
		env.reset();
	});
	destroy_thread.join();

	if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
	                             std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	return viewer_fully_connected.load(std::memory_order_acquire) ? kScenarioLateRegistrationAccepted : kScenarioSuccess;
}

int RunShutdownAdmissionClosesBeforeWorkerJoinScenario(testing::TestNodeHandle *nh)
{
	auto env = std::make_unique<MujocoEnvTestWrapper>("", nh);
	env->StartPhysicsLoop();
	env->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool viewer_registered{ false };
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool late_connect_rejected{ false };
	std::atomic_bool viewer_fully_connected{ false };
	std::atomic_bool teardown_started{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread late_connect_thread([&env, &viewer_mutex, &connection_for_exit, &viewer_registered, &glfw_ready,
	                                 &late_connect_rejected, &viewer_fully_connected, &teardown_started,
	                                 &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env.get(), true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = viewer->connection_state_;
			}
			viewer_registered.store(true, std::memory_order_release);
			viewer->RenderLoop(
			    [&viewer_fully_connected]() { viewer_fully_connected.store(true, std::memory_order_release); });
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
			late_connect_rejected.store(false, std::memory_order_release);
		} catch (const std::runtime_error &error) {
			if (IsExpectedLateActivationRuntimeError(teardown_started, viewer_fully_connected, error)) {
				late_connect_rejected.store(true, std::memory_order_release);
			} else {
				render_result.store(kScenarioLifetimeFailed, std::memory_order_release);
			}
		} catch (...) {
			render_result.store(
			    ClassifyRenderThreadException({ &viewer_registered, &glfw_ready, &viewer_fully_connected }),
			    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, late_connect_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&viewer_registered]() { return viewer_registered.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioGlfwStartupFailed;
	}

	teardown_started.store(true, std::memory_order_release);
	env.reset();

	if (!JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, late_connect_thread, render_loop_exited,
	                             std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	return late_connect_rejected.load(std::memory_order_acquire) ? kScenarioSuccess : kScenarioLateRegistrationAccepted;
}

int RunViewerCreatedBeforeCloseRejectsLaterRenderLoopScenario(testing::TestNodeHandle *nh)
{
	auto env    = std::make_unique<MujocoEnvTestWrapper>("", nh);
	auto viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env.get(), true, false);
	env.reset();
	const std::atomic_bool teardown_started{ true };
	const std::atomic_bool fully_connected{ false };
	try {
		viewer->RenderLoop();
		return kScenarioLateRegistrationAccepted;
	} catch (const std::runtime_error &error) {
		return IsExpectedLateActivationRuntimeError(teardown_started, fully_connected, error) ? kScenarioSuccess :
		                                                                                        kScenarioLifetimeFailed;
	} catch (...) {
		return kScenarioLifetimeFailed;
	}
}

int RunGenericLateActivationRuntimeErrorScenario(testing::TestNodeHandle * /*nh*/)
{
	const std::atomic_bool viewer_constructed{ true };
	const std::atomic_bool glfw_ready{ false };
	const std::atomic_bool fully_connected{ false };
	const std::atomic_bool teardown_started{ true };
	try {
		throw std::runtime_error("unexpected generic failure");
	} catch (const std::runtime_error &error) {
		if (IsExpectedLateActivationRuntimeError(teardown_started, fully_connected, error)) {
			return kScenarioSuccess;
		}
		return kScenarioLifetimeFailed;
	} catch (...) {
		return ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &fully_connected });
	}
}

int RunReconnectedViewerDisablesHeadlessScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<HeadlessTestEnv>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &viewer_connected,
	                           &glfw_ready, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	const auto cleanup_render_thread = [&]() {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
	};

	if (!WaitUntil([&viewer_constructed]() { return viewer_constructed.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioGlfwStartupFailed;
	}
	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		cleanup_render_thread();
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		cleanup_render_thread();
		return kScenarioViewerNotConnected;
	}

	RequestViewerExit(connection_for_exit, &viewer_mutex);
	if (!WaitUntil([&render_loop_exited]() { return render_loop_exited.load(std::memory_order_acquire); },
	               std::chrono::seconds(10))) {
		cleanup_render_thread();
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_thread.joinable()) {
		render_thread.join();
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	if (!env->IsHeadless()) {
		return kScenarioLifetimeFailed;
	}

	{
		auto replacement = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
		if (env->IsHeadless()) {
			return kScenarioLifetimeFailed;
		}
	}

	return kScenarioSuccess;
}

// A render thread can still be mid GL-context creation (slow on this CI box)
// while holding a pointer into the environment. We cannot join it (it is stuck)
// and we cannot let the environment be destroyed while it runs (use-after-free).
// These child scenarios exit right after they return, so terminate the process
// directly with the failure code instead of returning -- returning would destroy
// the live thread and trigger std::terminate (the "terminate called without an
// active exception" crash).
[[noreturn]] static void ExitChildScenario(int code)
{
	::_exit(code);
}

int RunConcurrentVoluntaryDisconnectHeadlessScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<HeadlessTestEnv>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> first_connection_for_exit;
	std::shared_ptr<ViewerConnectionState> second_connection_for_exit;
	std::atomic_bool first_glfw_ready{ false };
	std::atomic_bool second_glfw_ready{ false };
	std::atomic_bool first_viewer_constructed{ false };
	std::atomic_bool second_viewer_constructed{ false };
	std::atomic_bool first_viewer_connected{ false };
	std::atomic_bool second_viewer_connected{ false };
	std::atomic_bool first_render_loop_exited{ false };
	std::atomic_bool second_render_loop_exited{ false };
	std::atomic_int first_render_result{ kScenarioSuccess };
	std::atomic_int second_render_result{ kScenarioSuccess };

	auto run_viewer = [&](std::shared_ptr<ViewerConnectionState> *connection_slot, std::atomic_bool *viewer_constructed,
	                      std::atomic_bool *viewer_connected, std::atomic_bool *glfw_ready,
	                      std::atomic_bool *render_loop_exited, std::atomic_int *render_result) {
		RenderLoopExitMarker exit_marker(*render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				*connection_slot = local_viewer->connection_state_;
			}
			viewer_constructed->store(true, std::memory_order_release);
			local_viewer->RenderLoop([viewer_connected, glfw_ready]() {
				viewer_connected->store(true, std::memory_order_release);
				glfw_ready->store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_slot->reset();
			}
		} catch (...) {
			render_result->store(ClassifyRenderThreadException({ viewer_constructed, glfw_ready, viewer_connected }),
			                     std::memory_order_release);
		}
	};

	std::thread first_render_thread(run_viewer, &first_connection_for_exit, &first_viewer_constructed,
	                                &first_viewer_connected, &first_glfw_ready, &first_render_loop_exited,
	                                &first_render_result);
	if (!WaitUntil([&first_viewer_constructed]() { return first_viewer_constructed.load(std::memory_order_acquire); },
	               std::chrono::seconds(5)) ||
	    !WaitUntil([&first_glfw_ready]() { return first_glfw_ready.load(std::memory_order_acquire); },
	               std::chrono::seconds(5)) ||
	    !WaitUntil([&first_viewer_connected]() { return first_viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		JoinRenderThreadBounded(first_connection_for_exit, &viewer_mutex, first_render_thread, first_render_loop_exited,
		                        std::chrono::seconds(10));
		ExitChildScenario(kScenarioGlfwStartupFailed);
	}

	std::thread second_render_thread(run_viewer, &second_connection_for_exit, &second_viewer_constructed,
	                                 &second_viewer_connected, &second_glfw_ready, &second_render_loop_exited,
	                                 &second_render_result);
	if (!WaitUntil([&second_viewer_constructed]() { return second_viewer_constructed.load(std::memory_order_acquire); },
	               std::chrono::seconds(5)) ||
	    !WaitUntil([&second_glfw_ready]() { return second_glfw_ready.load(std::memory_order_acquire); },
	               std::chrono::seconds(5)) ||
	    !WaitUntil([&second_viewer_connected]() { return second_viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		RequestViewerExit(second_connection_for_exit, &viewer_mutex);
		JoinRenderThreadBounded(first_connection_for_exit, &viewer_mutex, first_render_thread, first_render_loop_exited,
		                        std::chrono::seconds(10));
		JoinRenderThreadBounded(second_connection_for_exit, &viewer_mutex, second_render_thread,
		                        second_render_loop_exited, std::chrono::seconds(10));
		ExitChildScenario(kScenarioGlfwStartupFailed);
	}

	std::thread first_disconnect([&]() {
		std::shared_ptr<ViewerConnectionState> connection;
		{
			std::lock_guard<std::mutex> lock(viewer_mutex);
			connection = first_connection_for_exit;
		}
		RequestViewerExit(connection, nullptr);
	});
	std::thread second_disconnect([&]() {
		std::shared_ptr<ViewerConnectionState> connection;
		{
			std::lock_guard<std::mutex> lock(viewer_mutex);
			connection = second_connection_for_exit;
		}
		RequestViewerExit(connection, nullptr);
	});
	first_disconnect.join();
	second_disconnect.join();

	if (!JoinRenderThreadBounded(first_connection_for_exit, &viewer_mutex, first_render_thread, first_render_loop_exited,
	                             std::chrono::seconds(10)) ||
	    !JoinRenderThreadBounded(second_connection_for_exit, &viewer_mutex, second_render_thread,
	                             second_render_loop_exited, std::chrono::seconds(10))) {
		if (first_render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			ExitChildScenario(first_render_result.load(std::memory_order_acquire));
		}
		if (second_render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			ExitChildScenario(second_render_result.load(std::memory_order_acquire));
		}
		ExitChildScenario(kScenarioRenderThreadTimedOut);
	}
	if (first_render_result.load(std::memory_order_acquire) != kScenarioSuccess ||
	    second_render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return kScenarioLifetimeFailed;
	}
	if (!WaitUntil([&env]() { return !env->HasConnectedViewers(); }, std::chrono::seconds(5))) {
		return kScenarioLifetimeFailed;
	}
	if (!WaitUntil([&env]() { return env->IsHeadless(); }, std::chrono::seconds(5))) {
		return kScenarioLifetimeFailed;
	}
	return kScenarioSuccess;
}

int RunConnectedViewersLeasePinsViewerStorageScenario(testing::TestNodeHandle *nh)
{
	auto env                    = std::make_unique<LeaseTestEnv>("", nh);
	LeaseTestEnv *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool render_loop_exited{ false };
	std::optional<ConnectedViewersLease> lease;
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &viewer_connected,
	                           &glfw_ready, &render_loop_exited, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	if (!WaitUntil([&viewer_constructed]() { return viewer_constructed.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return kScenarioGlfwStartupFailed;
	}
	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return kScenarioViewerNotConnected;
	}

	lease.emplace(env_raw->TakeLease());
	if (lease->empty()) {
		lease.reset();
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return kScenarioViewerNotConnected;
	}

	RequestViewerExit(connection_for_exit, &viewer_mutex);
	if (WaitUntil([&render_loop_exited]() { return render_loop_exited.load(std::memory_order_acquire); },
	              std::chrono::milliseconds(300))) {
		lease.reset();
		JoinRenderThreadBounded(nullptr, nullptr, render_thread, render_loop_exited, std::chrono::seconds(10));
		return kScenarioLeaseDrainFinishedEarly;
	}

	lease.reset();
	if (!JoinRenderThreadBounded(nullptr, nullptr, render_thread, render_loop_exited, std::chrono::seconds(10))) {
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	return kScenarioSuccess;
}

int RunViewerDisconnectsBeforeEnvironmentDestructionScenario(testing::TestNodeHandle *nh)
{
	auto env                            = std::make_unique<MujocoEnvTestWrapper>("", nh);
	MujocoEnvTestWrapper *const env_raw = env.get();
	env_raw->StartEventLoop();

	std::mutex viewer_mutex;
	std::shared_ptr<ViewerConnectionState> connection_for_exit;
	std::atomic_bool glfw_ready{ false };
	std::atomic_bool viewer_constructed{ false };
	std::atomic_bool viewer_connected{ false };
	std::atomic_bool render_loop_exited{ false };
	std::atomic_bool env_destroyed{ false };
	std::atomic_bool scenario_finished{ false };
	std::atomic_int render_result{ kScenarioSuccess };

	std::thread render_thread([env_raw, &viewer_mutex, &connection_for_exit, &viewer_constructed, &viewer_connected,
	                           &glfw_ready, &render_loop_exited, &env_destroyed, &scenario_finished, &render_result]() {
		RenderLoopExitMarker exit_marker(render_loop_exited);
		try {
			auto local_viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env_raw, true, false);
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit = local_viewer->connection_state_;
			}
			viewer_constructed.store(true, std::memory_order_release);
			local_viewer->RenderLoop([&viewer_connected, &glfw_ready]() {
				glfw_ready.store(true, std::memory_order_release);
				viewer_connected.store(true, std::memory_order_release);
			});
			{
				std::lock_guard<std::mutex> lock(viewer_mutex);
				connection_for_exit.reset();
			}
			render_loop_exited.store(true, std::memory_order_release);
			if (!WaitUntil([&env_destroyed]() { return env_destroyed.load(std::memory_order_acquire); },
			               std::chrono::seconds(10))) {
				render_result.store(kScenarioDestroyTimedOut, std::memory_order_release);
				return;
			}
			local_viewer.reset();
			scenario_finished.store(true, std::memory_order_release);
		} catch (...) {
			render_result.store(ClassifyRenderThreadException({ &viewer_constructed, &glfw_ready, &viewer_connected }),
			                    std::memory_order_release);
		}
	});

	if (!WaitUntil([&viewer_constructed]() { return viewer_constructed.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return kScenarioGlfwStartupFailed;
	}
	if (!WaitUntil([&glfw_ready]() { return glfw_ready.load(std::memory_order_acquire); }, std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return ClassifyGlfwReadyWaitTimeout({ &viewer_constructed, &glfw_ready, &viewer_connected });
	}
	if (!WaitUntil([&viewer_connected]() { return viewer_connected.load(std::memory_order_acquire); },
	               std::chrono::seconds(5))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		return kScenarioViewerNotConnected;
	}

	RequestViewerExit(connection_for_exit, &viewer_mutex);
	if (!WaitUntil([&render_loop_exited]() { return render_loop_exited.load(std::memory_order_acquire); },
	               std::chrono::seconds(10))) {
		JoinRenderThreadBounded(connection_for_exit, &viewer_mutex, render_thread, render_loop_exited,
		                        std::chrono::seconds(10));
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioRenderThreadTimedOut;
	}

	env.reset();
	env_destroyed.store(true, std::memory_order_release);

	if (!WaitUntil([&scenario_finished]() { return scenario_finished.load(std::memory_order_acquire); },
	               std::chrono::seconds(10))) {
		JoinRenderThreadBounded(nullptr, nullptr, render_thread, render_loop_exited, std::chrono::seconds(10));
		if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
			return render_result.load(std::memory_order_acquire);
		}
		return kScenarioDestroyTimedOut;
	}
	if (render_thread.joinable()) {
		render_thread.join();
	}
	if (render_result.load(std::memory_order_acquire) != kScenarioSuccess) {
		return render_result.load(std::memory_order_acquire);
	}
	return kScenarioSuccess;
}

#define LIFETIME_CHILD_SCENARIO(Name, Runner)                      \
	TEST_F(BaseEnvFixture, Name##Child)                             \
	{                                                               \
		if (std::getenv("MUJOCO_ROS_LIFETIME_CHILD") == nullptr) {   \
			GTEST_SKIP() << "child-only scenario entrypoint";         \
		}                                                            \
		if (!NativeDisplayIsAdvertised()) {                          \
			::_exit(kScenarioGlfwStartupFailed);                      \
		}                                                            \
		try {                                                        \
			const int result = Runner(nh.get());                      \
			::_exit(result);                                          \
		} catch (...) {                                              \
			::_exit(kScenarioLifetimeFailed);                         \
		}                                                            \
	}                                                               \
	TEST_F(BaseEnvFixture, Name##Isolated)                          \
	{                                                               \
		if (!NativeDisplayIsAdvertised()) {                          \
			GTEST_SKIP() << "native display unavailable";             \
		}                                                            \
		ExpectChildScenarioSuccess("BaseEnvFixture." #Name "Child"); \
	}

#define LIFETIME_CHILD_ABORT_SCENARIO(Name, Runner)               \
	TEST_F(BaseEnvFixture, Name##Child)                            \
	{                                                              \
		if (std::getenv("MUJOCO_ROS_LIFETIME_CHILD") == nullptr) {  \
			GTEST_SKIP() << "child-only scenario entrypoint";        \
		}                                                           \
		if (!NativeDisplayIsAdvertised()) {                         \
			::_exit(kScenarioGlfwStartupFailed);                     \
		}                                                           \
		try {                                                       \
			const int result = Runner(nh.get());                     \
			if (result == kScenarioGlfwStartupFailed) {              \
				::_exit(kScenarioGlfwStartupFailed);                  \
			}                                                        \
			if (result == kScenarioViewerNotConnected) {             \
				::_exit(kScenarioViewerNotConnected);                 \
			}                                                        \
			if (result != kScenarioSuccess) {                        \
				::_exit(kScenarioLifetimeFailed);                     \
			}                                                        \
			::_exit(kScenarioLifetimeFailed);                        \
		} catch (...) {                                             \
			::_exit(kScenarioLifetimeFailed);                        \
		}                                                           \
	}                                                              \
	TEST_F(BaseEnvFixture, Name##Isolated)                         \
	{                                                              \
		if (!NativeDisplayIsAdvertised()) {                         \
			GTEST_SKIP() << "native display unavailable";            \
		}                                                           \
		ExpectChildScenarioAborts("BaseEnvFixture." #Name "Child"); \
	}

LIFETIME_CHILD_SCENARIO(ConnectedViewerLeaseReleaseAfterEnvironmentCloseIsBounded,
                        RunConnectedViewerLeaseReleaseScenario)
LIFETIME_CHILD_SCENARIO(ExternallyOwnedViewerSurvivesEnvironmentDestruction, RunExternallyOwnedViewerSurvivesScenario)
LIFETIME_CHILD_SCENARIO(ViewerDisconnectsBeforeEnvironmentDestruction,
                        RunViewerDisconnectsBeforeEnvironmentDestructionScenario)
LIFETIME_CHILD_ABORT_SCENARIO(ViewerDestructorDuringRenderLoopTerminates, RunViewerDestructorDuringRenderLoopScenario)
LIFETIME_CHILD_ABORT_SCENARIO(EnvironmentDestructorFromViewerThreadTerminates,
                              RunEnvironmentDestructorFromViewerThreadScenario)
LIFETIME_CHILD_SCENARIO(LeaseBlocksEnvironmentModelReset, RunLeaseBlocksEnvironmentModelResetScenario)
LIFETIME_CHILD_SCENARIO(LateViewerActivationIsRejectedAndRolledBack, RunLateViewerActivationRejectedScenario)
LIFETIME_CHILD_SCENARIO(ShutdownAdmissionClosesBeforeWorkerJoin, RunShutdownAdmissionClosesBeforeWorkerJoinScenario)
LIFETIME_CHILD_SCENARIO(ViewerCreatedBeforeCloseRejectsLaterRenderLoop,
                        RunViewerCreatedBeforeCloseRejectsLaterRenderLoopScenario)
LIFETIME_CHILD_SCENARIO(ReconnectedViewerDisablesHeadless, RunReconnectedViewerDisablesHeadlessScenario)
LIFETIME_CHILD_SCENARIO(ConcurrentVoluntaryDisconnectHeadless, RunConcurrentVoluntaryDisconnectHeadlessScenario)
LIFETIME_CHILD_SCENARIO(ConnectedViewersLeasePinsViewerStorage, RunConnectedViewersLeasePinsViewerStorageScenario)

TEST_F(BaseEnvFixture, GenericLateActivationRuntimeErrorRejectedChild)
{
	if (std::getenv("MUJOCO_ROS_LIFETIME_CHILD") == nullptr) {
		GTEST_SKIP() << "child-only scenario entrypoint";
	}
	if (!NativeDisplayIsAdvertised()) {
		::_exit(kScenarioGlfwStartupFailed);
	}
	try {
		const int result = RunGenericLateActivationRuntimeErrorScenario(nh.get());
		::_exit(result);
	} catch (...) {
		::_exit(kScenarioLifetimeFailed);
	}
}

TEST_F(BaseEnvFixture, GenericLateActivationRuntimeErrorRejectedIsolated)
{
	if (!NativeDisplayIsAdvertised()) {
		GTEST_SKIP() << "native display unavailable";
	}
	ExpectChildScenarioSuccess("BaseEnvFixture.GenericLateActivationRuntimeErrorRejectedChild", kScenarioLifetimeFailed);
}

#undef LIFETIME_CHILD_SCENARIO
#undef LIFETIME_CHILD_ABORT_SCENARIO

class LateViewerConnectDuringReloadEnv : public MujocoEnvTestWrapper
{
public:
	using MujocoEnvTestWrapper::MujocoEnvTestWrapper;
	using MujocoEnvTestWrapper::RequestReload;
	ConnectedViewersLease TakeLeaseForTest() const { return AcquireConnectedViewersLease(); }
	bool ReloadInProgressForTest() const { return reload_in_progress_.load(std::memory_order_acquire); }
};

// Regression test for a deadlock between a viewer's own first-connect
// handshake and a concurrently in-flight reload's tail notification:
//
//   - LoadWithModelAndData() sets reload_in_progress_ = true, then (at its
//     tail) hands an async Load() to every viewer AcquireConnectedViewersLease()
//     considers "connected" -- blocking until that viewer's RenderLoop services it.
//   - MujocoEnv::ConnectViewer()'s own retry loop refuses to proceed with its
//     first-time InitializeModel() while reload_in_progress_ is true, and just
//     keeps retrying.
//
// Before the fix, a viewer that was merely *registered* (via CreateViewerConnection)
// but still stuck inside its own ConnectViewer() retry loop was nonetheless handed
// the async Load() -- and nothing could ever service it, since that viewer's
// RenderLoop() hadn't reached its main frame loop yet. Both sides waited on each
// other forever. The fix (ViewerConnectionState::render_loop_ready_, set only once
// ConnectViewer() has resolved) makes AcquireConnectedViewersLease() skip such a
// viewer -- it doesn't need the notification anyway, since it will pick up the
// current model itself via InitializeModel() as soon as reload_in_progress_ clears.
TEST_F(BaseEnvFixture, LateConnectingViewerDuringReloadDoesNotDeadlock)
{
	if (!NativeDisplayIsAdvertised()) {
		GTEST_SKIP() << "native display unavailable";
	}

	// Racing the very first model load against the very first viewer
	// connection is a faithful reproduction of the original bug, which was
	// specifically about server-startup ordering.
	auto env = std::make_unique<LateViewerConnectDuringReloadEnv>("", nh.get());

	std::mutex sync_mutex;
	std::condition_variable sync_cv;
	bool reload_reached_tail = false;
	bool release_reload      = false;
	std::promise<void> reload_finished;
	auto reload_finished_future = reload_finished.get_future();

	// kRenderReconfigureStarted fires right before LoadWithModelAndData()'s tail
	// (AcquireConnectedViewersLease + viewer->Load()) -- block there so the
	// about-to-be-created viewer below has time to reach ConnectViewer()'s retry
	// loop while reload_in_progress_ is still true, exactly as in the real race.
	env->SetReloadObserver([&](MujocoEnv::ReloadPhase phase) {
		if (phase == MujocoEnv::ReloadPhase::kRenderReconfigureStarted) {
			std::unique_lock<std::mutex> lock(sync_mutex);
			reload_reached_tail = true;
			sync_cv.notify_all();
			sync_cv.wait(lock, [&] { return release_reload; });
		} else if (phase == MujocoEnv::ReloadPhase::kNewGenerationLoaded) {
			reload_finished.set_value();
		}
	});

	// Non-blocking: this is the very first model load, racing the very first
	// viewer connection below -- exactly the server-startup ordering that
	// triggered the original deadlock.
	env->StartWithXML(testing::get_test_model_path("empty_world.xml"), false);
	{
		std::unique_lock<std::mutex> lock(sync_mutex);
		ASSERT_TRUE(sync_cv.wait_for(lock, std::chrono::seconds(2), [&] { return reload_reached_tail; }))
		    << "initial load did not reach kRenderReconfigureStarted";
	}
	ASSERT_TRUE(env->ReloadInProgressForTest());

	// The viewer itself: its ConnectViewer() must retry against
	// reload_in_progress_ == true right now, exactly like the real bug.
	// RenderLoop() runs on THIS (the test's main) thread below, matching
	// main.cpp's own usage -- running it on a spawned thread instead fails
	// GLFW context creation in this environment. A background "releaser"
	// thread does the lease inspection and unblocks the reload while
	// RenderLoop() is stuck retrying here.
	auto viewer = std::make_unique<Viewer>(std::make_unique<GlfwAdapter>(), env.get(), true, false);

	bool lease_excluded_connecting_viewer = false;
	std::thread releaser_thread([&] {
		// Give the viewer's ConnectViewer() retry loop a moment to actually
		// start spinning against the still-open reload before inspecting it.
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		const auto lease                 = env->TakeLeaseForTest();
		const auto matches               = std::any_of(lease.viewers().begin(), lease.viewers().end(),
		                                               [&](Viewer *leased_viewer) { return leased_viewer == viewer.get(); });
		lease_excluded_connecting_viewer = !matches;

		// Release the reload. Before the fix, its tail would have blocked
		// forever on this viewer's Load() future, and this viewer's
		// ConnectViewer() would never see reload_in_progress_ clear -- the
		// RenderLoop() call below would then never return.
		{
			std::lock_guard<std::mutex> lock(sync_mutex);
			release_reload = true;
		}
		sync_cv.notify_all();
	});

	// on_ready fires once ConnectViewer() has actually resolved -- exit right
	// away, there is nothing else for this test to do with a running viewer.
	ASSERT_NO_THROW(viewer->RenderLoop([&] { viewer->exit_request.store(1); }));

	releaser_thread.join();
	EXPECT_TRUE(lease_excluded_connecting_viewer)
	    << "a viewer still inside its own ConnectViewer() handshake must not be handed an async Load() -- nothing "
	       "can service it yet, and doing so deadlocks the reload against this viewer's own connect";
	ASSERT_EQ(reload_finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready)
	    << "reload did not complete -- deadlocked against the connecting viewer";

	env->shutdown();
}

} // namespace
#endif

namespace mujoco_ros {

class SimulationControlStateTestAccess
{
public:
	class ScopedAdmissionTestHook
	{
	public:
		ScopedAdmissionTestHook(SimulationControlState::AdmissionTestHook hook, void *context)
		{
			SetAdmissionTestHook(hook, context);
		}

		~ScopedAdmissionTestHook() { SetAdmissionTestHook(nullptr, nullptr); }

		ScopedAdmissionTestHook(const ScopedAdmissionTestHook &)            = delete;
		ScopedAdmissionTestHook &operator=(const ScopedAdmissionTestHook &) = delete;
	};

	static bool IsAdmissionTransactionLocked(SimulationControlState &state)
	{
		if (!state.state_mutex_.try_lock()) {
			return true;
		}
		state.state_mutex_.unlock();
		return false;
	}

	static bool IsManualStepAdmissionPoint(int hook_point)
	{
		return hook_point == static_cast<int>(SimulationControlState::AdmissionTestHookPoint::kBeforeManualStepAdmission);
	}

private:
	static void SetAdmissionTestHook(SimulationControlState::AdmissionTestHook hook, void *context)
	{
		SimulationControlState::admission_test_hook_context_.store(context);
		SimulationControlState::admission_test_hook_.store(hook);
	}
};

} // namespace mujoco_ros

namespace {

class AdmissionArbitrationGate
{
public:
	static void PauseManualStepAdmission(void *context, int hook_point)
	{
		if (!SimulationControlStateTestAccess::IsManualStepAdmissionPoint(hook_point)) {
			return;
		}
		auto &gate = *static_cast<AdmissionArbitrationGate *>(context);
		std::unique_lock<std::mutex> lock(gate.mutex_);
		gate.manual_step_checkpoint_reached_ = true;
		gate.condition_.notify_all();
		gate.condition_.wait(lock, [&gate] { return gate.manual_step_checkpoint_released_; });
	}

	bool WaitForManualStepCheckpoint()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return condition_.wait_for(lock, std::chrono::seconds(1), [this] { return manual_step_checkpoint_reached_; });
	}

	void ReleaseManualStepCheckpoint()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		manual_step_checkpoint_released_ = true;
		condition_.notify_all();
	}

private:
	std::mutex mutex_;
	std::condition_variable condition_;
	bool manual_step_checkpoint_reached_  = false;
	bool manual_step_checkpoint_released_ = false;
};

class LifecycleTransitionProbe
{
public:
	void MarkStarted()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		started_ = true;
		condition_.notify_all();
	}

	void MarkCompleted()
	{
		std::lock_guard<std::mutex> lock(mutex_);
		completed_ = true;
		condition_.notify_all();
	}

	bool WaitUntilStarted()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return condition_.wait_for(lock, std::chrono::seconds(1), [this] { return started_; });
	}

	bool WaitUntilCompleted()
	{
		std::unique_lock<std::mutex> lock(mutex_);
		return condition_.wait_for(lock, std::chrono::seconds(1), [this] { return completed_; });
	}

private:
	std::mutex mutex_;
	std::condition_variable condition_;
	bool started_   = false;
	bool completed_ = false;
};

const char *LifecycleRequestName(int request)
{
	switch (request) {
		case 0:
			return "reset";
		case 1:
			return "load";
		case 2:
			return "shutdown";
		default:
			return "run";
	}
}

void ApplyLifecycleRequest(SimulationControlState &state, int request)
{
	switch (request) {
		case 0:
			state.RequestReset();
			break;
		case 1:
			state.SetLoadRequest(1);
			break;
		case 2:
			state.RequestShutdown();
			break;
		default:
			state.SetPaused(false);
			break;
	}
}

void ExpectLifecycleRequestActive(const SimulationControlSnapshot &snapshot, int request)
{
	switch (request) {
		case 0:
			EXPECT_TRUE(snapshot.reset_requested);
			break;
		case 1:
			EXPECT_EQ(snapshot.load_request, 1);
			break;
		case 2:
			EXPECT_TRUE(snapshot.shutdown_requested);
			break;
		default:
			EXPECT_TRUE(snapshot.running);
			break;
	}
}

} // namespace

TEST(SimulationControlStateTest, PauseAndRunSnapshot)
{
	SimulationControlState state;

	EXPECT_FALSE(state.Snapshot().running);
	state.SetPaused(false);
	EXPECT_TRUE(state.Snapshot().running);
	state.SetPaused(true);
	EXPECT_FALSE(state.Snapshot().running);
}

TEST(SimulationControlStateTest, RejectsInvalidManualStepRequests)
{
	SimulationControlState state;

	EXPECT_FALSE(state.RequestSteps(0));
	EXPECT_FALSE(state.RequestSteps(-1));

	state.SetPaused(false);
	EXPECT_FALSE(state.RequestSteps(1));
}

TEST(SimulationControlStateTest, RejectsOverlappingManualStepRequests)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	EXPECT_FALSE(state.RequestSteps(1));
	EXPECT_EQ(state.Snapshot().pending_steps, 3);
}

TEST(SimulationControlStateTest, RejectsManualStepsForEveryConflictingRequestState)
{
	SimulationControlState reset_state;
	reset_state.RequestReset();
	EXPECT_FALSE(reset_state.RequestSteps(1));

	SimulationControlState load_state;
	load_state.SetLoadRequest(1);
	EXPECT_FALSE(load_state.RequestSteps(1));

	SimulationControlState shutdown_state;
	shutdown_state.RequestShutdown();
	EXPECT_FALSE(shutdown_state.RequestSteps(1));

	SimulationControlState running_state;
	running_state.SetPaused(false);
	EXPECT_FALSE(running_state.RequestSteps(1));

	SimulationControlState pending_state;
	ASSERT_TRUE(pending_state.RequestSteps(1));
	EXPECT_FALSE(pending_state.RequestSteps(1));
}

TEST(SimulationControlStateTest, ConcurrentManualStepRequestsHaveExactlyOneWinner)
{
	constexpr int kRequests = 16;
	SimulationControlState state;
	std::atomic_int ready    = { 0 };
	std::atomic_bool start   = { false };
	std::atomic_int admitted = { 0 };
	std::vector<std::thread> requesters;
	requesters.reserve(kRequests);

	for (int i = 0; i < kRequests; ++i) {
		requesters.emplace_back([&] {
			ready.fetch_add(1);
			while (!start.load()) {
			}
			if (state.RequestSteps(1)) {
				admitted.fetch_add(1);
			}
		});
	}
	while (ready.load() != kRequests) {
	}
	start.store(true);
	for (auto &requester : requesters) {
		requester.join();
	}

	EXPECT_EQ(admitted.load(), 1);
	EXPECT_EQ(state.Snapshot().pending_steps, 1);
}

TEST(SimulationControlStateTest, ConcurrentLifecycleRequestNeverLeavesPendingManualSteps)
{
	constexpr int kAttempts = 1024;
	for (int request = 0; request < 4; ++request) {
		for (int attempt = 0; attempt < kAttempts; ++attempt) {
			SimulationControlState state;
			std::atomic_int ready  = { 0 };
			std::atomic_bool start = { false };
			std::thread stepper([&] {
				ready.fetch_add(1);
				while (!start.load()) {
				}
				state.RequestSteps(1);
			});
			std::thread lifecycle([&] {
				ready.fetch_add(1);
				while (!start.load()) {
				}
				switch (request) {
					case 0:
						state.RequestReset();
						break;
					case 1:
						state.SetLoadRequest(1);
						break;
					case 2:
						state.RequestShutdown();
						break;
					default:
						state.SetPaused(false);
						break;
				}
			});
			while (ready.load() != 2) {
			}
			start.store(true);
			stepper.join();
			lifecycle.join();

			const auto snapshot = state.Snapshot();
			const bool conflict =
			    snapshot.reset_requested || snapshot.load_request > 0 || snapshot.shutdown_requested || snapshot.running;
			EXPECT_FALSE(conflict && snapshot.pending_steps > 0) << "request=" << request << ", attempt=" << attempt;
		}
	}
}

TEST(SimulationControlStateTest, LifecycleRequestsCannotBeOverwrittenByManualStepAdmissionRace)
{
	for (int request = 0; request < 4; ++request) {
		SCOPED_TRACE(LifecycleRequestName(request));
		SimulationControlState state;
		AdmissionArbitrationGate gate;
		SimulationControlStateTestAccess::ScopedAdmissionTestHook hook_scope(
		    AdmissionArbitrationGate::PauseManualStepAdmission, &gate);

		std::atomic_bool manual_step_admitted = { false };
		std::thread stepper([&] { manual_step_admitted.store(state.RequestSteps(1)); });

		if (!gate.WaitForManualStepCheckpoint()) {
			stepper.join();
			FAIL() << "manual-step admission hook was not reached";
			return;
		}

		const bool admission_transaction_locked = SimulationControlStateTestAccess::IsAdmissionTransactionLocked(state);
		EXPECT_TRUE(admission_transaction_locked) << "manual-step lifecycle check and pending-step store must be atomic";

		LifecycleTransitionProbe lifecycle_probe;
		std::thread lifecycle([&] {
			lifecycle_probe.MarkStarted();
			ApplyLifecycleRequest(state, request);
			lifecycle_probe.MarkCompleted();
		});

		if (!lifecycle_probe.WaitUntilStarted()) {
			gate.ReleaseManualStepCheckpoint();
			stepper.join();
			lifecycle.join();
			FAIL() << "lifecycle request thread did not start";
			return;
		}

		if (!admission_transaction_locked && !lifecycle_probe.WaitUntilCompleted()) {
			gate.ReleaseManualStepCheckpoint();
			stepper.join();
			lifecycle.join();
			FAIL() << "pre-fix lifecycle request did not complete before stale manual-step store";
			return;
		}

		gate.ReleaseManualStepCheckpoint();
		stepper.join();
		lifecycle.join();

		const auto snapshot = state.Snapshot();
		EXPECT_TRUE(manual_step_admitted.load());
		ExpectLifecycleRequestActive(snapshot, request);
		EXPECT_EQ(snapshot.pending_steps, 0) << "lifecycle request must cancel a manual step admitted from a stale check";
	}
}

TEST(SimulationControlStateTest, ManualStepProgressIsSnapshotBased)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(2));
	EXPECT_EQ(state.Snapshot().pending_steps, 2);
	EXPECT_TRUE(state.RecordCompletedStep());
	EXPECT_EQ(state.Snapshot().pending_steps, 1);
	EXPECT_TRUE(state.RecordCompletedStep());
	EXPECT_EQ(state.Snapshot().pending_steps, 0);
	EXPECT_FALSE(state.RecordCompletedStep());
}

TEST(SimulationControlStateTest, LifecycleCancellationWinsOverStepCompletionSnapshot)
{
	SimulationControlState state;
	ManualStepToken token = 0;
	ASSERT_TRUE(state.RequestSteps(2, &token));
	EXPECT_EQ(state.GetManualStepSnapshot(token).status, ManualStepTerminalStatus::kPending);

	state.RequestReset();
	const auto terminal = state.GetManualStepSnapshot(token);
	EXPECT_EQ(terminal.status, ManualStepTerminalStatus::kCancelled);
	EXPECT_EQ(terminal.pending_steps, 0);
	EXPECT_FALSE(state.RecordCompletedStep());
	state.AcknowledgeManualStep(token);
}

TEST(SimulationControlStateTest, LoadingWindowTransitionsAreExplicit)
{
	SimulationControlState state;

	EXPECT_EQ(state.Snapshot().model_lifecycle, ModelLifecyclePhase::kNoModel);
	state.SetLoadRequest(1);
	EXPECT_EQ(state.Snapshot().model_lifecycle, ModelLifecyclePhase::kLoading);

	state.SetLifecyclePhase(ModelLifecyclePhase::kOperational);
	EXPECT_EQ(state.Snapshot().model_lifecycle, ModelLifecyclePhase::kOperational);

	state.RequestShutdown();
	EXPECT_EQ(state.Snapshot().model_lifecycle, ModelLifecyclePhase::kShuttingDown);
	state.SetLifecyclePhase(ModelLifecyclePhase::kOperational);
	EXPECT_EQ(state.Snapshot().model_lifecycle, ModelLifecyclePhase::kShuttingDown);
}

TEST(SimulationControlStateTest, LoadRequestClearPreservesLifecycleUntilExplicitPublish)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	state.SetLoadRequest(0);
	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kLoading)
	    << "SetLoadRequest(0) must not publish kNoModel; failure paths use CompleteFailedLoad";
}

TEST(SimulationControlStateTest, PublishOperationalIdleEnablesStepAdmission)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	EXPECT_FALSE(state.RequestSteps(1));
	state.PublishOperationalIdle();
	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kOperational);
	EXPECT_TRUE(state.RequestSteps(1));
}

TEST(SimulationControlStateTest, CompleteFailedLoadPublishesNoModelAtomically)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	state.CompleteFailedLoad();
	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kNoModel);
}

TEST(SimulationControlStateTest, CompleteFailedLoadRespectsShutdownLifecycle)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	state.RequestShutdown();
	state.CompleteFailedLoad();
	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kShuttingDown);
}

TEST(SimulationControlStateTest, CompleteFailedLoadCancelsOperationalManualSteps)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	state.PublishOperationalIdle();
	ManualStepToken token = 0;
	ASSERT_TRUE(state.RequestSteps(2, &token));
	EXPECT_EQ(state.Snapshot().pending_steps, 2);

	state.CompleteFailedLoad();

	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.pending_steps, 0);
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kNoModel);
	EXPECT_EQ(state.GetManualStepSnapshot(token).status, ManualStepTerminalStatus::kCancelled);
	state.AcknowledgeManualStep(token);
}

TEST(SimulationControlStateTest, PublishOperationalIdleClearsLoadRequestDuringShutdown)
{
	SimulationControlState state;
	state.SetLoadRequest(2);
	state.RequestShutdown();
	state.PublishOperationalIdle();
	const auto snapshot = state.Snapshot();
	EXPECT_EQ(snapshot.load_request, 0);
	EXPECT_EQ(snapshot.model_lifecycle, ModelLifecyclePhase::kShuttingDown);
}

TEST(SimulationControlStateTest, ManualStepWaitUsesTerminalConditionWithoutPolling)
{
	SimulationControlState state;
	ManualStepToken token = 0;
	ASSERT_TRUE(state.RequestSteps(1, &token));
	std::promise<ManualStepSnapshot> waiter;
	auto result = waiter.get_future();
	std::thread wait_thread([&] { waiter.set_value(state.WaitForManualStepUpdate(token, 1)); });

	state.RequestShutdown();
	wait_thread.join();
	EXPECT_EQ(result.get().status, ManualStepTerminalStatus::kCancelled);
	state.AcknowledgeManualStep(token);
}

TEST(SimulationControlStateTest, UnpauseClearsPendingManualSteps)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	state.SetPaused(false);

	const auto snapshot = state.Snapshot();
	EXPECT_TRUE(snapshot.running);
	EXPECT_EQ(snapshot.pending_steps, 0);
}

TEST(SimulationControlStateTest, LifecycleRequestsClearPendingManualSteps)
{
	SimulationControlState state;

	ASSERT_TRUE(state.RequestSteps(3));
	state.RequestReset();
	EXPECT_TRUE(state.Snapshot().reset_requested);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);

	EXPECT_FALSE(state.RequestSteps(2));
	state.ClearResetRequest();
	ASSERT_TRUE(state.RequestSteps(2));
	state.SetLoadRequest(2);
	EXPECT_EQ(state.Snapshot().load_request, 2);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);

	EXPECT_FALSE(state.RequestSteps(1));
	state.SetLoadRequest(0);
	ASSERT_TRUE(state.RequestSteps(1));
	state.RequestShutdown();
	EXPECT_TRUE(state.Snapshot().shutdown_requested);
	EXPECT_EQ(state.Snapshot().pending_steps, 0);
	EXPECT_FALSE(state.RequestSteps(1));
}

TEST(SimulationControlStateTest, ClearsLifecycleAndConsumesSpeedChange)
{
	SimulationControlState state;

	state.RequestReset();
	state.ClearResetRequest();
	EXPECT_FALSE(state.Snapshot().reset_requested);

	state.SetLoadRequest(2);
	state.SetLoadRequest(1);
	EXPECT_EQ(state.Snapshot().load_request, 1);
	state.SetLoadRequest(0);
	EXPECT_EQ(state.Snapshot().load_request, 0);

	state.MarkSpeedChanged();
	EXPECT_TRUE(state.Snapshot().speed_changed);
	EXPECT_TRUE(state.ConsumeSpeedChange());
	EXPECT_FALSE(state.Snapshot().speed_changed);
	EXPECT_FALSE(state.ConsumeSpeedChange());
}

TEST_F(BaseEnvFixture, ControlRequestsUseAuthoritativeLoadState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->requestLoad(2);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 2);

	sync_env->requestLoad(1);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 1);

	sync_env->requestLoad(3);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 3);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, RenderBackpressurePolicyRejectsInvalidValuesAtomically)
{
	auto sync_env = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	EXPECT_EQ(sync_env->GetRenderBackpressurePolicy(), rendering::RenderBackpressurePolicy::kDrop);
	ASSERT_TRUE(sync_env->SetRenderBackpressurePolicy("wait_for_slot").ok());
	EXPECT_EQ(sync_env->GetRenderBackpressurePolicy(), rendering::RenderBackpressurePolicy::kWaitForSlot);

	const auto rejected = sync_env->SetRenderBackpressurePolicy("wait");
	EXPECT_EQ(rejected.code, rendering::FrameStatusCode::kInvalidPolicy);
	EXPECT_EQ(sync_env->GetRenderBackpressurePolicy(), rendering::RenderBackpressurePolicy::kWaitForSlot);
}

TEST_F(BaseEnvFixture, FrameSlotWarningsAreThrottledAndStatusSpecific)
{
	auto sync_env     = std::make_unique<WarningTestWrapper>("", nh.get());
	auto warning_time = std::chrono::steady_clock::time_point(std::chrono::seconds(10));
	sync_env->SetWarningClockForTesting([&warning_time] { return warning_time; });
	const rendering::FrameStatus dropped{ rendering::FrameStatusCode::kFrameSlotsExhausted, 0, std::nullopt,
		                                   FrameGeneration(1), "frame storage exhausted" };
	const rendering::FrameStatus cancelled{ rendering::FrameStatusCode::kStopped, 0, std::nullopt, FrameGeneration(1),
		                                     "capacity wait cancelled" };
	const rendering::FrameStatus backend_failure{ rendering::FrameStatusCode::kBackendFailure, 0, std::nullopt,
		                                           FrameGeneration(1), "backend failed" };

	sync_env->WarnFrameSlotDrop(cancelled);
	EXPECT_EQ(sync_env->FrameSlotWarningCountForTesting(), 0U);
	sync_env->WarnFrameSlotDrop(dropped);
	sync_env->WarnFrameSlotDrop(dropped);
	EXPECT_EQ(sync_env->FrameSlotWarningCountForTesting(), 1U);
	EXPECT_NE(sync_env->LastFrameSlotWarningForTesting().find("render_backpressure_policy=wait_for_slot"),
	          std::string::npos);
	warning_time += std::chrono::milliseconds(999);
	sync_env->WarnFrameSlotDrop(dropped);
	EXPECT_EQ(sync_env->FrameSlotWarningCountForTesting(), 1U);
	warning_time += std::chrono::milliseconds(1);
	sync_env->WarnFrameSlotDrop(dropped);
	EXPECT_EQ(sync_env->FrameSlotWarningCountForTesting(), 2U);
	sync_env->WarnFrameSlotDrop(backend_failure);
	EXPECT_EQ(sync_env->FrameSlotWarningCountForTesting(), 2U);
}

TEST_F(BaseEnvFixture, ManualStepAdmissionRejectsMissingModel)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->SetPaused(true);

	EXPECT_FALSE(sync_env->RequestManualSteps(1));
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 0);
	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, DirectSettingsLifecycleMutationCannotBypassControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetPaused(true);
	EXPECT_FALSE(sync_env->GetControlSnapshot().running);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 0);
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 0);

	// EnvSettings no longer contains lifecycle fields. This test compiles only against
	// configuration/internal markers, proving callers must use MujocoEnv control methods.
	sync_env->requestLoad(2);
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 2);
	EXPECT_FALSE(sync_env->GetControlSnapshot().running);
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 0);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ManualStepProgressDoesNotReapplyCompletedStepCount)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->StartWithXML("<mujoco/>");
	sync_env->SetPaused(true);
	ASSERT_FALSE(sync_env->GetControlSnapshot().running);

	ASSERT_TRUE(sync_env->RequestManualSteps(2));
	sync_env->RecordCompletedManualStep();
	ASSERT_EQ(sync_env->GetControlSnapshot().pending_steps, 1);

	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 1);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerControlRequestsRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->StartWithXML("<mujoco/>");

	sync_env->SetPaused(false);
	EXPECT_TRUE(sync_env->GetControlSnapshot().running);

	sync_env->SetPaused(true);
	EXPECT_FALSE(sync_env->GetControlSnapshot().running);

	EXPECT_TRUE(sync_env->RequestManualSteps(2));
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 2);

	sync_env->RequestViewerReset();
	EXPECT_TRUE(sync_env->GetControlSnapshot().reset_requested);
	EXPECT_EQ(sync_env->GetControlSnapshot().pending_steps, 0);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerLoadAndShutdownRequestsRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->RequestReload();
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 3);

	sync_env->RequestViewerShutdown();
	EXPECT_TRUE(sync_env->GetControlSnapshot().shutdown_requested);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, LoadPayloadPublishesBeforeRequestInsideControlBoundary)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	std::atomic_int payload_observed_load_request       = { -1 };
	std::atomic_bool payload_ran_under_boundary         = { false };
	std::atomic_bool payload_ran_under_physics_boundary = { false };
	const std::string queued_name                       = "queued-by-load-payload";

	sync_env->PublishLoadRequestForTest(1, [&] {
		payload_observed_load_request.store(sync_env->GetControlSnapshot().load_request);
		mju::strcpy_arr(sync_env->queued_filename_, queued_name.c_str());

		std::promise<bool> can_acquire_boundary;
		auto boundary_result = can_acquire_boundary.get_future();
		std::thread inspector([&] { can_acquire_boundary.set_value(sync_env->CanAcquireControlBoundaryForTest()); });
		inspector.join();
		payload_ran_under_boundary.store(!boundary_result.get());

		std::promise<bool> can_acquire_physics_boundary;
		auto physics_boundary_result = can_acquire_physics_boundary.get_future();
		std::thread physics_inspector(
		    [&] { can_acquire_physics_boundary.set_value(sync_env->CanAcquirePhysicsBoundaryForTest()); });
		physics_inspector.join();
		payload_ran_under_physics_boundary.store(!physics_boundary_result.get());
	});

	EXPECT_EQ(payload_observed_load_request.load(), 0) << "payload must run before load-request publication";
	EXPECT_TRUE(payload_ran_under_boundary.load())
	    << "payload and load-request publication must share one control boundary";
	EXPECT_TRUE(payload_ran_under_physics_boundary.load())
	    << "payload publication must share the event-loop load-consumption boundary";
	EXPECT_STREQ(sync_env->queued_filename_, queued_name.c_str());
	EXPECT_EQ(sync_env->GetControlSnapshot().load_request, 1);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, ViewerSpeedChangesRouteThroughControlState)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetViewerRealTimeIndex(3);
	EXPECT_TRUE(sync_env->GetControlSnapshot().speed_changed);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, SpeedSnapshotPairsIndexWithChangeConsumption)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	sync_env->SetViewerRealTimeIndex(4);
	auto first_snapshot = sync_env->ConsumeSpeedSettingsSnapshotForTest();
	EXPECT_EQ(first_snapshot.real_time_index, 4);
	EXPECT_TRUE(first_snapshot.speed_changed);
	EXPECT_FALSE(sync_env->GetControlSnapshot().speed_changed);

	auto second_snapshot = sync_env->ConsumeSpeedSettingsSnapshotForTest();
	EXPECT_EQ(second_snapshot.real_time_index, 4);
	EXPECT_FALSE(second_snapshot.speed_changed);

	sync_env->SetViewerRealTimeIndex(7);
	auto third_snapshot = sync_env->ConsumeSpeedSettingsSnapshotForTest();
	EXPECT_EQ(third_snapshot.real_time_index, 7);
	EXPECT_TRUE(third_snapshot.speed_changed);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, EvalModeWithoutHashThrow)
{
	MJR_WARN("###### [START] EvalModeWithoutHashThrow ######");
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	EXPECT_THROW(env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get()), std::runtime_error);
	MJR_WARN("###### [END] EvalModeWithoutHashThrow ######");
}

TEST_F(BaseEnvFixture, RunEvalMode)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().shutdown_requested) << "Exit request is set before shutdown!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalPauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	env_ptr->togglePaused(true, "some_hash");
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalPauseWithoutHashFails)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should start running!";

	EXPECT_FALSE(env_ptr->togglePaused(true)) << "Pause without admin hash should fail in eval mode!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should keep running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalUnpauseWithHash)
{
	nh->setParam("eval_mode", true);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	env_ptr->togglePaused(false, "some_hash");
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, EvalUnpauseWithoutHash)
{
	nh->setParam("eval_mode", true);
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("some_hash", nh.get());

	env_ptr->StartWithXML(xml_path);

	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should start paused!";

	EXPECT_TRUE(env_ptr->togglePaused(false)) << "Unpause without admin hash should succeed in eval mode!";
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should be running!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepBeforeLoad)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	EXPECT_FALSE(env_ptr->step(1));
}

TEST_F(BaseEnvFixture, StepAfterShutdown)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->shutdown();
	EXPECT_FALSE(env_ptr->step(1));
}

TEST_F(BaseEnvFixture, StepWhileUnpaused)
{
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_FALSE(env_ptr->step(1));

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepSingleWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(1));
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepMultiWhilePaused)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(100));
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 100 * env_ptr->getModelPtr()->opt.timestep, 1e-6);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepUnblocked)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_TRUE(env_ptr->step(100, false));
	EXPECT_GT(env_ptr->GetControlSnapshot().pending_steps, 0);

	float seconds = 0;
	while (env_ptr->getDataPtr()->time < 100 * env_ptr->getModelPtr()->opt.timestep && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Time should have passed but ran into 2 seconds timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepNegativeFail)
{
	nh->setParam("unpause", false);
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);
	EXPECT_FALSE(env_ptr->step(-10)) << "Stepping with negative steps should not succeed!";
	EXPECT_EQ(env_ptr->GetControlSnapshot().pending_steps, 0);
	EXPECT_DOUBLE_EQ(env_ptr->getDataPtr()->time, 0.0);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Shutdown)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	EXPECT_FALSE(env_ptr->isPhysicsRunning()) << "Physics thread should not be running yet!";
	EXPECT_FALSE(env_ptr->isEventRunning()) << "Event thread should not be running yet!";

	env_ptr->StartPhysicsLoop();
	env_ptr->StartEventLoop();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().shutdown_requested) << "Exit request is set before shutdown!";

	// Make sure the threads are running
	float seconds = 0;
	while (seconds < 2 && (!env_ptr->isPhysicsRunning() || !env_ptr->isEventRunning())) { // wait for threads to start
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_TRUE(env_ptr->isPhysicsRunning()) << "Physics thread should have started by now!";
	EXPECT_TRUE(env_ptr->isEventRunning()) << "Event thread should have started by now!";

	env_ptr->Shutdown();

	seconds = 0;
	while (seconds < 2 && (env_ptr->isPhysicsRunning() || env_ptr->isEventRunning())) { // wait for threads to exit
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_FALSE(env_ptr->isPhysicsRunning()) << "Physics thread is still running after shutdown!";
	EXPECT_FALSE(env_ptr->isEventRunning()) << "Event thread is still running after shutdown!";

	env_ptr->WaitForEventsJoin();
	env_ptr->WaitForPhysicsJoin();
}

TEST_F(BaseEnvFixture, InitWithModel)
{
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

	seconds = 0;
	while (env_ptr->getDataPtr()->time == 0 && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time did not pass in simulation, ran into 2 second timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RuntimeOptionsTransactionsAdvanceEpochAndRollback)
{
	nh->setParam("unpause", false);
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	env_ptr->StartWithXML(testing::get_test_model_path("empty_world.xml"));

	const auto before = env_ptr->GetRuntimeOptions();
	ASSERT_TRUE(before.ok());

	const auto applied = env_ptr->ApplyRuntimeOptions({ { "timestep", 0.002 }, { "iterations", std::int64_t(50) } });
	ASSERT_TRUE(applied.ok());
	ASSERT_TRUE(applied.effective.has_value());
	EXPECT_EQ(applied.epoch.value(), before.epoch.value() + 1);
	EXPECT_DOUBLE_EQ(applied.effective->timestep, 0.002);
	EXPECT_EQ(applied.effective->iterations, 50);

	const auto rejected =
	    env_ptr->ApplyRuntimeOptions({ { "timestep", 0.003 }, { "solimp", std::string("0.9 0.95 0.001 0.5 nan") } });
	ASSERT_FALSE(rejected.ok());
	ASSERT_TRUE(rejected.error.has_value());
	EXPECT_EQ(rejected.error->field, "solimp");
	EXPECT_EQ(rejected.epoch, applied.epoch);

	const auto after = env_ptr->GetRuntimeOptions();
	ASSERT_TRUE(after.ok());
	EXPECT_EQ(after.epoch, applied.epoch);
	EXPECT_EQ(after.effective, applied.effective);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, RuntimeOptionsUpdateWaitsForHeldPhysicsBoundary)
{
	nh->setParam("unpause", false);
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->StartWithXML(testing::get_test_model_path("empty_world.xml"), false);
	ASSERT_TRUE(sync_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));

	sync_env->getMutexPtr()->lock();
	std::promise<bool> admission_probe;
	std::promise<RuntimeOptionsTransactionResult> transaction;
	auto admission_result   = admission_probe.get_future();
	auto transaction_result = transaction.get_future();
	std::thread updater([&] {
		admission_probe.set_value(!sync_env->CanAcquirePhysicsBoundaryForTest());
		transaction.set_value(sync_env->ApplyRuntimeOptions({ { "timestep", 0.002 } }));
	});

	const bool update_blocked = admission_result.get();
	if (!update_blocked) {
		sync_env->getMutexPtr()->unlock();
		static_cast<void>(transaction_result.get());
		updater.join();
		FAIL() << "The held physics boundary must block the update thread";
	}
	EXPECT_EQ(transaction_result.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);

	sync_env->getMutexPtr()->unlock();
	const auto applied = transaction_result.get();
	updater.join();
	ASSERT_TRUE(applied.ok());
	ASSERT_TRUE(applied.effective.has_value());
	EXPECT_DOUBLE_EQ(applied.effective->timestep, 0.002);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, RuntimeOptionsRejectsEveryLoadingWindowStage)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());

	for (const int request : { 3, 2, 1 }) {
		sync_env->requestLoad(request);
		ASSERT_EQ(sync_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kLoading);
		EXPECT_THROW(sync_env->SetPendingRuntimeOptions({ { "timestep", 0.002 } }), std::runtime_error);
		const auto rejected = sync_env->ApplyRuntimeOptions({ { "timestep", 0.002 } });
		EXPECT_FALSE(rejected.ok());
		ASSERT_TRUE(rejected.error.has_value());
		EXPECT_EQ(rejected.error->message, "Runtime Options unavailable during Loading Window");
	}

	sync_env->requestLoad(0);
	EXPECT_EQ(sync_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kLoading);
	EXPECT_FALSE(sync_env->ApplyRuntimeOptions({ { "timestep", 0.002 } }).ok());

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, RejectedReloadUpdateDoesNotReachReplacementModel)
{
	nh->setParam("unpause", false);
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->StartWithXML(testing::get_test_model_path("empty_world.xml"), false);
	ASSERT_TRUE(sync_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));
	const auto before = sync_env->GetRuntimeOptions();
	ASSERT_TRUE(before.ok());

	const std::string replacement = testing::get_test_model_path("pendulum_world.xml");
	sync_env->getMutexPtr()->lock();
	sync_env->PublishLoadRequestForTest(3, [&] { mju::strcpy_arr(sync_env->queued_filename_, replacement.c_str()); });
	ASSERT_EQ(sync_env->GetControlSnapshot().model_lifecycle, ModelLifecyclePhase::kLoading);
	const auto rejected = sync_env->ApplyRuntimeOptions({ { "timestep", 0.002 } });
	ASSERT_FALSE(rejected.ok());

	sync_env->requestLoad(2);
	sync_env->getMutexPtr()->unlock();
	ASSERT_TRUE(sync_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));
	ASSERT_EQ(sync_env->getFilename(), replacement);

	const auto after = sync_env->GetRuntimeOptions();
	ASSERT_TRUE(after.ok());
	EXPECT_EQ(after.effective, before.effective);
	EXPECT_DOUBLE_EQ(after.effective->timestep, 0.001);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, FailedLoadClearsStartupRuntimeOptions)
{
	auto sync_env = std::make_unique<ControlStateTestWrapper>("", nh.get());
	sync_env->SetPendingRuntimeOptions({ { "timestep", 0.002 } });
	sync_env->StartWithXML("<mujoco>", false);
	ASSERT_TRUE(sync_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));
	EXPECT_FALSE(sync_env->sim_state_.model_valid);

	sync_env->StartWithXML(testing::get_test_model_path("empty_world.xml"), false);
	ASSERT_TRUE(sync_env->WaitForOperationalStatusIdle(std::chrono::seconds(2)));
	const auto options = sync_env->GetRuntimeOptions();
	ASSERT_TRUE(options.ok());
	EXPECT_DOUBLE_EQ(options.effective->timestep, 0.001);

	sync_env->shutdown();
}

TEST_F(BaseEnvFixture, PauseUnpause)
{
	nh->setParam("unpause", false);
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";

	mjtNum time = env_ptr->getDataPtr()->time;

	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	EXPECT_EQ(env_ptr->getDataPtr()->time, time) << "Time should not have changed in paused mode!";

	ASSERT_TRUE(env_ptr->togglePaused(false));

	float seconds = 0;
	while (env_ptr->getDataPtr()->time == time && seconds < 2) { // wait for model to be loaded or timeout
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Time should have been moving forward in unpaused state, ran into 2 seconds timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, StepsTerminate)
{
	nh->setParam("num_steps", 100);

	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	float seconds = 0;
	int current;
	int last = env_ptr->getPendingSteps();
	while (env_ptr->getPendingSteps() > 0) {
		current = env_ptr->getPendingSteps();
		if (current == last) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
			seconds += 0.002;
		} else {
			last = current;
		}
		if (seconds >= 2)
			break;
		seconds = 0.;
	}
	EXPECT_LT(seconds, 2) << "Pending steps should have decreased but ran into 2 seconds timeout";

	EXPECT_NEAR(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep * 100,
	            env_ptr->getModelPtr()->opt.timestep * 0.1)
	    << "Time should have stopped after 100 steps";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, ManualSteps)
{
	nh->setParam("unpause", false);

	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_FALSE(env_ptr->GetControlSnapshot().pending_steps)
	    << "pending manual steps should be 0 after initialization!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should not be running!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, 0) << "Time should be 0 after initialization!";

	EXPECT_TRUE(env_ptr->Step(1, false));

	float seconds = 0;
	while (env_ptr->GetControlSnapshot().pending_steps != 0 && seconds < 1) { // wait for step completion
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		seconds += 0.002;
	}
	EXPECT_LT(seconds, 1) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, env_ptr->getModelPtr()->opt.timestep)
	    << "Time should have been increased by one step!";

	EXPECT_TRUE(env_ptr->TogglePaused(false));
	EXPECT_FALSE(env_ptr->Step(100, false));

	// Wait for time to pass
	std::this_thread::sleep_for(std::chrono::milliseconds(2));

	EXPECT_EQ(env_ptr->GetControlSnapshot().pending_steps, 0)
	    << "pending manual steps should stay clear in unpaused mode!";
	EXPECT_TRUE(env_ptr->TogglePaused(true));

	mjtNum time = env_ptr->getDataPtr()->time;

	EXPECT_TRUE(env_ptr->Step(100, false));

	seconds = 0;
	while (env_ptr->GetControlSnapshot().pending_steps != 0 && seconds < 2) { // wait for step completion
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		seconds += 0.005;
	}
	EXPECT_LT(seconds, 2) << "Manual step should have been executed but ran into 1 second timeout!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, time + 100 * env_ptr->getModelPtr()->opt.timestep,
	            env_ptr->getModelPtr()->opt.timestep * 0.1)
	    << "Time should have been increased by 100*timestep!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, Reset)
{
	nh->setParam("unpause", false);
	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->StartWithXML(xml_path);

	EXPECT_TRUE(env_ptr->step(100)) << "Stepping failed!";

	ASSERT_TRUE(env_ptr->togglePaused(true));
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 100 * env_ptr->getModelPtr()->opt.timestep, 1e-6)
	    << "Time should have been running!";

	env_ptr->Reset();

	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused after reset!";
	EXPECT_NEAR(env_ptr->getDataPtr()->time, 0, 1e-6) << "Time should have been reset to 0!";

	ASSERT_TRUE(env_ptr->togglePaused(false));
	env_ptr->Reset();
	EXPECT_TRUE(env_ptr->GetControlSnapshot().running) << "Model should keep running after reset!";

	ASSERT_TRUE(env_ptr->togglePaused(true));
	int id2 = mujoco_ros::util::jointName2id(env_ptr->getModelPtr(), "joint2");
	EXPECT_NE(id2, -1) << "joint2 should exist in model!";
	env_ptr->getDataPtr()->qpos[env_ptr->getModelPtr()->jnt_qposadr[id2]] = 0.5;
	env_ptr->getDataPtr()->qvel[env_ptr->getModelPtr()->jnt_dofadr[id2]]  = 0.1;
	env_ptr->Reset();
	EXPECT_NE(env_ptr->getDataPtr()->qpos[id2], 0.5) << "joint2 position should have been reset!";
	EXPECT_NE(env_ptr->getDataPtr()->qvel[id2], 0.1) << "joint2 velocity should have been reset!";

	env_ptr->shutdown();
}

// Test reloading
TEST_F(BaseEnvFixture, Reload)
{
	nh->setParam("unpause", false);

	env_ptr              = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	std::string xml_path = testing::get_test_model_path("empty_world.xml");
	env_ptr->StartWithXML(xml_path);

	// Load same model again in unpaused state
	env_ptr->load_queued_model();
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused on init!";
	EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Wrong content in filename_!";
	EXPECT_EQ(env_ptr->getDataPtr()->time, 0) << "Time should have been reset to 0!";
	EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Model should stay paused after reset!";

	// Load new model in paused state
	std::string xml_path2 = testing::get_test_model_path("pendulum_world.xml");
	env_ptr->load_filename(xml_path2);
	EXPECT_EQ(env_ptr->getFilename(), xml_path2) << "Wrong content in filename_!";

	ASSERT_TRUE(env_ptr->togglePaused(false));

	// Let some time pass
	float seconds = 0;
	while (env_ptr->getDataPtr()->time < 0.01 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Simulation time did not advance before timeout!";

	// Load same model in unpaused state
	env_ptr->load_queued_model();
	EXPECT_EQ(env_ptr->getFilename(), xml_path2) << "Wrong content in filename_!";

	// Let some time pass
	seconds = 0;
	while (env_ptr->getDataPtr()->time < 0.01 && seconds < 2) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		seconds += 0.001;
	}
	EXPECT_LT(seconds, 2) << "Simulation time did not advance before timeout!";

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, InitModelFromQueuedBuffer)
{
	nh->setParam("unpause", false);
	nh->setParam("realtime", 0.5);

	// Create a MujocoEnv object
	env_ptr       = std::make_unique<ControlStateTestWrapper>("", nh.get());
	auto sync_env = static_cast<ControlStateTestWrapper *>(env_ptr.get());
	static_cast<void>(sync_env->ConsumeSpeedSettingsSnapshotForTest());

	// Set the queued model buffer
	std::string queuedFilename = "<mujoco/>";

	// Call the initModelFromQueue function
	env_ptr->StartWithXML(queuedFilename);

	// Check the result
	ASSERT_TRUE(env_ptr->getModelPtr());
	ASSERT_TRUE(env_ptr->getDataPtr());
	ASSERT_STREQ(env_ptr->getFilename().c_str(), queuedFilename.c_str());
	ASSERT_TRUE(env_ptr->sim_state_.model_valid);
	auto speed_snapshot = sync_env->ConsumeSpeedSettingsSnapshotForTest();
	ASSERT_FLOAT_EQ(env_ptr->percentRealTime[speed_snapshot.real_time_index], 50.f);
	ASSERT_TRUE(speed_snapshot.speed_changed);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, InitModelFromInvalidQueuedBuffer)
{
	// Create a MujocoEnv object
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

	// Set the queued model buffer
	std::string valid = "<mujoco/>";

	// Call the initModelFromQueue function
	env_ptr->StartWithXML(valid);

	std::string invalid = "<mujoco>";
	env_ptr->load_filename(invalid);

	float seconds = 0;
	while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model load attempt
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
		seconds += 0.003;
	}
	EXPECT_LT(seconds, 2) << "Invalid model load did not finish before timeout!";

	// Check the result
	ASSERT_TRUE(env_ptr->getModelPtr());
	ASSERT_TRUE(env_ptr->getDataPtr());
	ASSERT_STREQ(env_ptr->getFilename().c_str(), valid.c_str());
	ASSERT_FALSE(env_ptr->sim_state_.model_valid);

	env_ptr->shutdown();
}

TEST_F(BaseEnvFixture, DestructorClearsGlobalInstancePointer)
{
	env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());
	ASSERT_EQ(mujoco_ros::MujocoEnv::instance, env_ptr.get());
	env_ptr->shutdown();
	env_ptr.reset();
	EXPECT_EQ(mujoco_ros::MujocoEnv::instance, nullptr)
	    << "Destroying a MujocoEnv must clear the global instance pointer (and the mjcb_control/mjcb_passive "
	       "callbacks that read it), otherwise a later mj_step/mj_compile use-after-frees the destroyed env.";
}

TEST_F(BaseEnvFixture, FromDescriptionProducesARunningEnv)
{
	env_ptr.reset(); // BaseEnvFixture's TearDown calls env_ptr->shutdown(); from_description returns a plain
	                 // MujocoEnv, not a MujocoEnvTestWrapper, so manage its lifetime directly in this test.
	auto env = mujoco_ros::MujocoEnv::from_description(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                                   std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf");
	ASSERT_NE(env, nullptr);
	EXPECT_TRUE(env->sim_state_.model_valid);
}

TEST_F(BaseEnvFixture, FromDescriptionThrowsOnMissingUrdf)
{
	env_ptr.reset();
	EXPECT_THROW(mujoco_ros::MujocoEnv::from_description(std::string(TEST_RESOURCES_DIR) + "/does_not_exist.urdf",
	                                                     std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf"),
	             std::runtime_error);
}

namespace {

constexpr mjtNum kValidMass                = 0.5;
constexpr mjtNum kValidIpos[3]             = { 0.01, 0.02, 0.03 };
constexpr mjtNum kValidPrincipalInertia[3] = { 1.0e-4, 2.0e-4, 3.0e-4 };
constexpr mjtNum kValidIquat[4]            = { 0.7071067811865476, 0.7071067811865476, 0.0, 0.0 };

void SetBodyBallQposOffset(MujocoEnvTestWrapper &env)
{
	const int body_id     = mj_name2id(env.getModelPtr(), mjOBJ_BODY, "body_ball");
	const int jnt_adr     = env.getModelPtr()->body_jntadr[body_id];
	const int jnt_qposadr = env.getModelPtr()->jnt_qposadr[jnt_adr];
	mjtNum pose[7]        = { 1.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0 };
	mju_normalize4(pose + 3);
	mju_copy(env.getDataPtr()->qpos + jnt_qposadr, pose, 7);
}

} // namespace

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesUpdatesModelAndPreservesQpos)
{
	mjModel *m        = env_ptr->getModelPtr();
	mjData *d         = env_ptr->getDataPtr();
	const int body_id = mj_name2id(m, mjOBJ_BODY, "immovable");
	ASSERT_GE(body_id, 1);

	SetBodyBallQposOffset(*env_ptr);
	std::vector<mjtNum> qpos_before(m->nq);
	mju_copy(qpos_before.data(), d->qpos, m->nq);

	EXPECT_TRUE(
	    env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia, kValidIquat));

	EXPECT_DOUBLE_EQ(m->body_mass[body_id], kValidMass);
	EXPECT_DOUBLE_EQ(m->body_ipos[body_id * 3 + 0], kValidIpos[0]);
	EXPECT_DOUBLE_EQ(m->body_ipos[body_id * 3 + 1], kValidIpos[1]);
	EXPECT_DOUBLE_EQ(m->body_ipos[body_id * 3 + 2], kValidIpos[2]);
	EXPECT_DOUBLE_EQ(m->body_inertia[body_id * 3 + 0], kValidPrincipalInertia[0]);
	EXPECT_DOUBLE_EQ(m->body_inertia[body_id * 3 + 1], kValidPrincipalInertia[1]);
	EXPECT_DOUBLE_EQ(m->body_inertia[body_id * 3 + 2], kValidPrincipalInertia[2]);
	EXPECT_DOUBLE_EQ(m->body_iquat[body_id * 4 + 0], kValidIquat[0]);
	EXPECT_DOUBLE_EQ(m->body_iquat[body_id * 4 + 1], kValidIquat[1]);
	EXPECT_DOUBLE_EQ(m->body_iquat[body_id * 4 + 2], kValidIquat[2]);
	EXPECT_DOUBLE_EQ(m->body_iquat[body_id * 4 + 3], kValidIquat[3]);
	for (int i = 0; i < m->nq; ++i) {
		EXPECT_DOUBLE_EQ(d->qpos[i], qpos_before[i]) << "qpos[" << i << "] changed after mj_setConst";
	}
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsMissingBody)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("no_such_body", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                kValidIquat, "", status, sizeof(status)));
	EXPECT_NE(std::string(status).find("no_such_body"), std::string::npos);
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsEmptyBodyName)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("", kValidMass, kValidIpos, kValidPrincipalInertia, kValidIquat, "",
	                                                status, sizeof(status)));
	EXPECT_NE(std::string(status).find("empty"), std::string::npos);
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNullPointers)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, nullptr, kValidPrincipalInertia,
	                                                kValidIquat, "", status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, nullptr, kValidIquat, "",
	                                                status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia, nullptr,
	                                                "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNonFiniteMass)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", std::numeric_limits<mjtNum>::infinity(), kValidIpos,
	                                                kValidPrincipalInertia, kValidIquat, "", status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", std::numeric_limits<mjtNum>::quiet_NaN(), kValidIpos,
	                                                kValidPrincipalInertia, kValidIquat, "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNonPositiveMass)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", 0.0, kValidIpos, kValidPrincipalInertia, kValidIquat,
	                                                "", status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", -1.0, kValidIpos, kValidPrincipalInertia, kValidIquat,
	                                                "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNonFiniteCoM)
{
	char status[MujocoEnv::kErrorLength] = {};
	const mjtNum bad_ipos[3]             = { std::numeric_limits<mjtNum>::quiet_NaN(), 0.0, 0.0 };
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, bad_ipos, kValidPrincipalInertia,
	                                                kValidIquat, "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsInvalidInertia)
{
	char status[MujocoEnv::kErrorLength] = {};
	const mjtNum zero_inertia[3]         = { 0.0, 1.0e-4, 1.0e-4 };
	const mjtNum negative_inertia[3]     = { -1.0e-4, 1.0e-4, 1.0e-4 };
	const mjtNum nan_inertia[3]          = { std::numeric_limits<mjtNum>::quiet_NaN(), 1.0e-4, 1.0e-4 };
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, zero_inertia, kValidIquat, "",
	                                                status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, negative_inertia, kValidIquat,
	                                                "", status, sizeof(status)));
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, nan_inertia, kValidIquat, "",
	                                                status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNonNormalizedQuaternion)
{
	char status[MujocoEnv::kErrorLength] = {};
	const mjtNum bad_quat[4]             = { 1.0, 1.0, 0.0, 0.0 };
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                bad_quat, "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNonFiniteQuaternion)
{
	char status[MujocoEnv::kErrorLength] = {};
	const mjtNum bad_quat[4]             = { 1.0, std::numeric_limits<mjtNum>::infinity(), 0.0, 0.0 };
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                bad_quat, "", status, sizeof(status)));
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsUnauthorizedHashInEvalMode)
{
	env_ptr->setEvalMode(true);
	env_ptr->setAdminHash("required_hash");
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                kValidIquat, "wrong_hash", status, sizeof(status)));
	EXPECT_NE(std::string(status).find("Unauthorized"), std::string::npos);
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsNegativeStatusBufferSize)
{
	char status[MujocoEnv::kErrorLength] = {};
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                kValidIquat, "", status, -1));
	EXPECT_EQ(status[0], '\0');
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesResolvesBodyUnderPhysicsLock)
{
	mjModel *m        = env_ptr->getModelPtr();
	const int body_id = mj_name2id(m, mjOBJ_BODY, "immovable");
	ASSERT_GE(body_id, 1);

	std::unique_lock<MujocoEnvMutex> physics_lock(*env_ptr->getMutexPtr());
	EXPECT_TRUE(
	    env_ptr->SetBodyInertialProperties("immovable", kValidMass, kValidIpos, kValidPrincipalInertia, kValidIquat));
	EXPECT_DOUBLE_EQ(m->body_mass[body_id], kValidMass);
}

TEST_F(PendulumEnvFixture, SetBodyInertialPropertiesRejectsMissingBodyUnderPhysicsLock)
{
	char status[MujocoEnv::kErrorLength] = {};
	std::unique_lock<MujocoEnvMutex> physics_lock(*env_ptr->getMutexPtr());
	EXPECT_FALSE(env_ptr->SetBodyInertialProperties("no_such_body", kValidMass, kValidIpos, kValidPrincipalInertia,
	                                                kValidIquat, "", status, sizeof(status)));
	EXPECT_NE(std::string(status).find("no_such_body"), std::string::npos);
}
