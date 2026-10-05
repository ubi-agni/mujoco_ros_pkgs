/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2026, Bielefeld University
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

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace mujoco_ros {

using ManualStepToken = std::uint64_t;

enum class ManualStepTerminalStatus
{
	kPending,
	kCompleted,
	kCancelled,
	kUnknown,
};

enum class ModelLifecyclePhase
{
	kNoModel,
	kLoading,
	kOperational,
	kShuttingDown,
};

struct ManualStepSnapshot
{
	ManualStepTerminalStatus status = ManualStepTerminalStatus::kUnknown;
	int pending_steps               = 0;
};

struct SimulationControlSnapshot
{
	bool running                        = false;
	int pending_steps                   = 0;
	bool shutdown_requested             = false;
	int load_request                    = 0;
	bool reset_requested                = false;
	bool speed_changed                  = false;
	int real_time_index                 = 9;
	ModelLifecyclePhase model_lifecycle = ModelLifecyclePhase::kNoModel;
};

class SimulationControlStateTestAccess;

class SimulationControlState
{
public:
	void SetPaused(bool paused)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		running_.store(!paused);
		if (running_) {
			CancelPendingStepsLocked();
		}
		state_condition_.notify_all();
	}

	bool RequestSteps(int num_steps, ManualStepToken *token = nullptr)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (shutdown_requested_.load() || reset_requested_.load() || load_request_.load() > 0 || running_.load() ||
		    pending_steps_.load() > 0 || num_steps <= 0) {
			return false;
		}

		InvokeAdmissionTestHook(AdmissionTestHookPoint::kBeforeManualStepAdmission);
		const ManualStepToken request_token = next_manual_step_token_++;
		active_manual_step_token_           = request_token;
		active_manual_step_tracked_         = token != nullptr;
		pending_steps_.store(num_steps);
		if (token != nullptr) {
			*token = request_token;
		}
		state_condition_.notify_all();
		return true;
	}

	bool HasPendingSteps() const
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		return pending_steps_.load() > 0;
	}

	bool RecordCompletedStep()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		int pending_steps = pending_steps_.load();
		if (pending_steps <= 0) {
			return false;
		}
		pending_steps_.store(pending_steps - 1);
		if (pending_steps == 1) {
			if (active_manual_step_tracked_) {
				terminal_manual_steps_[active_manual_step_token_] = ManualStepTerminalStatus::kCompleted;
			}
			active_manual_step_token_   = 0;
			active_manual_step_tracked_ = false;
		}
		state_condition_.notify_all();
		return true;
	}

	bool CancelPendingSteps(ManualStepToken token = 0)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (token != 0 && token != active_manual_step_token_) {
			return false;
		}
		return CancelPendingStepsLocked();
	}

	void RequestShutdown()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		shutdown_requested_.store(true);
		model_lifecycle_.store(ModelLifecyclePhase::kShuttingDown);
		CancelPendingStepsLocked();
	}

	bool IsShutdownRequested() const
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		return shutdown_requested_.load();
	}

	void SetLoadRequest(int load_request)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (load_request < 0) {
			load_request = 0;
		}
		if (load_request > 0) {
			model_lifecycle_.store(ModelLifecyclePhase::kLoading);
			CancelPendingStepsLocked();
		}
		load_request_.store(load_request);
		state_condition_.notify_all();
	}

	void CompleteFailedLoad()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		load_request_.store(0);
		CancelPendingStepsLocked();
		if (!shutdown_requested_.load()) {
			model_lifecycle_.store(ModelLifecyclePhase::kNoModel);
		}
		state_condition_.notify_all();
	}

	void PublishOperationalIdle()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		load_request_.store(0);
		if (!shutdown_requested_.load()) {
			model_lifecycle_.store(ModelLifecyclePhase::kOperational);
		}
		state_condition_.notify_all();
	}

	void SetLifecyclePhase(ModelLifecyclePhase phase)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		if (shutdown_requested_.load() && phase != ModelLifecyclePhase::kShuttingDown) {
			return;
		}
		model_lifecycle_.store(phase);
		state_condition_.notify_all();
	}

	void RequestReset()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		reset_requested_.store(true);
		CancelPendingStepsLocked();
	}

	void ClearResetRequest()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		reset_requested_.store(false);
		state_condition_.notify_all();
	}

	void SetRealTimeIndex(int real_time_index)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		real_time_index_.store(real_time_index);
		state_condition_.notify_all();
	}

	void MarkSpeedChanged()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		speed_changed_.store(true);
		state_condition_.notify_all();
	}

	bool ConsumeSpeedChange()
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		bool expected = true;
		return speed_changed_.compare_exchange_strong(expected, false);
	}

	SimulationControlSnapshot Snapshot() const
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		return SimulationControlSnapshot{ running_.load(),         pending_steps_.load(),   shutdown_requested_.load(),
			                               load_request_.load(),    reset_requested_.load(), speed_changed_.load(),
			                               real_time_index_.load(), model_lifecycle_.load() };
	}

	bool WaitForChangeUntil(std::chrono::steady_clock::time_point deadline) const
	{
		std::unique_lock<std::mutex> lock(state_mutex_);
		return state_condition_.wait_until(lock, deadline) != std::cv_status::timeout;
	}

	ManualStepSnapshot GetManualStepSnapshot(ManualStepToken token) const
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		return GetManualStepSnapshotLocked(token);
	}

	ManualStepSnapshot WaitForManualStepUpdate(ManualStepToken token, int observed_pending_steps) const
	{
		std::unique_lock<std::mutex> lock(state_mutex_);
		state_condition_.wait(lock, [this, token, observed_pending_steps] {
			const auto snapshot = GetManualStepSnapshotLocked(token);
			return snapshot.status != ManualStepTerminalStatus::kPending ||
			       snapshot.pending_steps != observed_pending_steps;
		});
		return GetManualStepSnapshotLocked(token);
	}

	void AcknowledgeManualStep(ManualStepToken token)
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		terminal_manual_steps_.erase(token);
	}

private:
	friend class SimulationControlStateTestAccess;

	enum class AdmissionTestHookPoint
	{
		kBeforeManualStepAdmission,
	};
	using AdmissionTestHook = void (*)(void *, int);

	bool CancelPendingStepsLocked()
	{
		if (pending_steps_.load() == 0) {
			return false;
		}
		if (active_manual_step_tracked_) {
			terminal_manual_steps_[active_manual_step_token_] = ManualStepTerminalStatus::kCancelled;
		}
		pending_steps_.store(0);
		active_manual_step_token_   = 0;
		active_manual_step_tracked_ = false;
		state_condition_.notify_all();
		return true;
	}

	ManualStepSnapshot GetManualStepSnapshotLocked(ManualStepToken token) const
	{
		if (token == 0) {
			return {};
		}
		if (token == active_manual_step_token_ && pending_steps_.load() > 0) {
			return { ManualStepTerminalStatus::kPending, pending_steps_.load() };
		}
		const auto terminal = terminal_manual_steps_.find(token);
		if (terminal != terminal_manual_steps_.end()) {
			return { terminal->second, 0 };
		}
		return {};
	}

	void InvokeAdmissionTestHook(AdmissionTestHookPoint point)
	{
		auto *hook = admission_test_hook_.load();
		if (hook) {
			hook(admission_test_hook_context_.load(), static_cast<int>(point));
		}
	}

	mutable std::mutex state_mutex_;
	mutable std::condition_variable state_condition_;
	static inline std::atomic<AdmissionTestHook> admission_test_hook_ = { nullptr };
	static inline std::atomic<void *> admission_test_hook_context_    = { nullptr };
	ManualStepToken next_manual_step_token_                           = 1;
	ManualStepToken active_manual_step_token_                         = 0;
	bool active_manual_step_tracked_                                  = false;
	std::unordered_map<ManualStepToken, ManualStepTerminalStatus> terminal_manual_steps_;
	std::atomic_bool running_                         = { false };
	std::atomic_int pending_steps_                    = { 0 };
	std::atomic_bool shutdown_requested_              = { false };
	std::atomic_int load_request_                     = { 0 };
	std::atomic_bool reset_requested_                 = { false };
	std::atomic_bool speed_changed_                   = { false };
	std::atomic_int real_time_index_                  = { 9 };
	std::atomic<ModelLifecyclePhase> model_lifecycle_ = { ModelLifecyclePhase::kNoModel };
};

} // namespace mujoco_ros
