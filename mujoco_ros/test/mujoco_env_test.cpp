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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/simulation_control_state.hpp>
#include <mujoco_ros/util.hpp>

#include <limits>

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
