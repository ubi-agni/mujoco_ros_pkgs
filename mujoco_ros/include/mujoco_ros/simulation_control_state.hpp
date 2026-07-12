/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2026, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

namespace mujoco_ros {

struct SimulationControlSnapshot
{
	bool running            = false;
	int pending_steps       = 0;
	bool shutdown_requested = false;
	int load_request        = 0;
	bool reset_requested    = false;
	bool speed_changed      = false;
};

class SimulationControlState
{
public:
	void SetPaused(bool paused)
	{
		running_.store(!paused);
		if (running_) {
			pending_steps_.store(0);
		}
	}

	bool RequestSteps(int num_steps)
	{
		if (shutdown_requested_.load() || running_.load() || pending_steps_.load() > 0 || num_steps <= 0) {
			return false;
		}

		pending_steps_.store(num_steps);
		return true;
	}

	bool HasPendingSteps() const { return pending_steps_.load() > 0; }

	bool RecordCompletedStep()
	{
		int pending_steps = pending_steps_.load();
		while (pending_steps > 0) {
			if (pending_steps_.compare_exchange_weak(pending_steps, pending_steps - 1)) {
				return true;
			}
		}

		return false;
	}

	void CancelPendingSteps() { pending_steps_.store(0); }

	void RequestShutdown()
	{
		shutdown_requested_.store(true);
		pending_steps_.store(0);
	}

	bool IsShutdownRequested() const { return shutdown_requested_.load(); }

	void SetLoadRequest(int load_request)
	{
		if (load_request < 0) {
			load_request = 0;
		}
		if (load_request > 0) {
			pending_steps_.store(0);
		}
		load_request_.store(load_request);
	}

	void RequestReset()
	{
		reset_requested_.store(true);
		pending_steps_.store(0);
	}

	void ClearResetRequest() { reset_requested_.store(false); }

	void MarkSpeedChanged() { speed_changed_.store(true); }

	bool ConsumeSpeedChange()
	{
		bool expected = true;
		return speed_changed_.compare_exchange_strong(expected, false);
	}

	SimulationControlSnapshot Snapshot() const
	{
		return SimulationControlSnapshot{ running_.load(),      pending_steps_.load(),   shutdown_requested_.load(),
			                               load_request_.load(), reset_requested_.load(), speed_changed_.load() };
	}

private:
	std::atomic_bool running_            = { false };
	std::atomic_int pending_steps_       = { 0 };
	std::atomic_bool shutdown_requested_ = { false };
	std::atomic_int load_request_        = { 0 };
	std::atomic_bool reset_requested_    = { false };
	std::atomic_bool speed_changed_      = { false };
};

} // namespace mujoco_ros
