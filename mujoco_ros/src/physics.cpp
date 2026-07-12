/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
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

/* Authors: David P. Leins */

#include <mujoco/mujoco.h>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/util.hpp>
#include <mujoco_ros/viewer.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
namespace roscpp = ros;
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
namespace roscpp = rclcpp;
#endif

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

void MujocoEnv::WrappedStep()
{
	mj_step(model_.get(), data_.get());
	ros_api_->PublishSimTime(data_->time);
	RunLastStageCbs();
	const char *message = Diverged(model_->opt.disableflags, data_.get());

	if (message) {
		MJR_WARN("Simulation diverged: %s", message);
		for (const auto &viewer : connected_viewers_) {
			mju::strcpy_arr(viewer->load_error, message);
		}
	}

	if (settings_.render_offscreen) {
		// Wait until no render request is pending
		while (offscreen_.request_pending.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(3));
		}
		std::unique_lock<std::mutex> lock(offscreen_.render_mutex);

		// Reset geoms created by the render callbacks otherwise they will be rendered again
		offscreen_.callbacks_scn.ngeom = 0;

		for (const auto &cam_ptr : offscreen_.cams) {
			roscpp::Time t = util::toRosTime(data_->time);
			if (cam_ptr->ShouldRender(t)) {
				mjv_copyModel(cam_ptr->model_state_, model_.get());
				mjv_copyData(cam_ptr->data_state_, cam_ptr->model_state_, data_.get());
				offscreen_.request_pending.store(true);
			}
			RunRenderCbs(&offscreen_.callbacks_scn);
		}
	}
	offscreen_.cond_render_request.notify_one();
}

void MujocoEnv::PhysicsLoop()
{
	MJR_DEBUG("Physics loop started");
	is_physics_running_ = 1;
	// CPU-sim syncronization point
	std::chrono::time_point<Clock> syncCPU;
	mjtNum syncSim = 0;

	// run until asked to exit
	while (roscpp::ok() && !IsShutdownRequested() && num_steps_until_exit_ != 0) {
		const auto control_snapshot = control_state_.Snapshot();
		// Sleep for 1 ms or yield, to let the main thread run
		// yield results in busy wait - which has better timing but kills battery life
		if (control_snapshot.running && settings_.busywait) {
			std::this_thread::yield();
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		// Run only if model is present
		if (!std::atomic_load(&model_))
			continue;

		// Try acquiring the sim mutex
		if (!physics_thread_mutex_.try_lock()) {
			// If mutex is locked, try again later
			continue;
		}

		const auto locked_control_snapshot = control_state_.Snapshot();

		// if simulation is paused
		if (!locked_control_snapshot.running) {
			SimPausedPhysics(syncSim);
		} else {
			SimUnpausedPhysics(syncSim, syncCPU);
		}
		// unlock physics mutex
		physics_thread_mutex_.unlock();
	}
	is_physics_running_ = 0;
	MJR_INFO_COND(num_steps_until_exit_ == 0, "Reached requested number of steps. Exiting simulation");
	if (num_steps_until_exit_ == 0) {
		RequestShutdown();
	}
	if (offscreen_.render_thread_handle.joinable()) {
		offscreen_.cond_render_request.notify_one();
		MJR_DEBUG("Joining offscreen render thread");
		offscreen_.render_thread_handle.join();
	}
	MJR_DEBUG("Exiting physics loop");
}

void MujocoEnv::SimPausedPhysics(mjtNum &syncSim)
{
	const auto startCPU = Clock::now();
	if (HasManualStepRequest()) { // Action call or arrow keys used for stepping
		syncSim = data_->time;

		while (HasManualStepRequest() &&
		       ( // connected_viewers_.empty() ||
		           Clock::now() - startCPU < Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_))) {
			// Run single step
			WrappedStep();

			RecordCompletedManualStep();
			// Break if reset
			if (data_->time < syncSim) {
				break;
			}
		}
	} else {
		// Run mj_forward, to update rendering and joint sliders
		mj_forward(model_.get(), data_.get());
		ros_api_->PublishSimTime(data_->time);
		// Sleep for the difference between the lower bound render rate (30Hz) and the time it took to run the forward
		// step to reduce cpu load
		std::this_thread::sleep_for(Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_) - (Clock::now() - startCPU));
	}
}

void MujocoEnv::SimUnpausedPhysics(mjtNum &syncSim, std::chrono::time_point<Clock> &syncCPU)
{
	// record CPU time at start of iteration
	const auto startCPU = Clock::now();

	// Elapsed CPU and simulation time since last sync
	const auto elapsedCPU = startCPU - syncCPU;
	double elapsedSim     = data_->time - syncSim;

	// Requested slow-down factor
	double slowdown = 100 / percentRealTime[settings_.real_time_index];

	// Misalignment condition: distance from target sim time is bigger than syncsimalign
	bool misaligned    = std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;
	bool speed_changed = ConsumeSpeedChange();

	// Out-of-sync (for any reason): reset sync times, step
	if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
	    speed_changed) {
		// re-sync
		syncCPU = startCPU;
		syncSim = data_->time;

		// run single step, let next iteration deal with timing
		WrappedStep();

		if (num_steps_until_exit_ > 0) {
			num_steps_until_exit_--;
		}
	}

	// In-sync: step until ahead of CPU
	else {
		bool measured  = false;
		mjtNum prevSim = data_->time;

		// If real-time is bound, run until sim steps are in sync with CPU steps, otherwise run as fast as
		// possible
		while ((settings_.real_time_index == 0 || Seconds((data_->time - syncSim) * slowdown) < Clock::now() - syncCPU) &&
		       (Clock::now() - startCPU < Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_) /*||
		        connected_viewers_.empty()*/) && // only break if rendering UI is actually necessary
		       !IsShutdownRequested() &&
		       num_steps_until_exit_ != 0 && control_state_.Snapshot().running) {
			// measure slowdown before first step
			if (!measured && elapsedSim) {
				if (settings_.real_time_index != 0) {
					sim_state_.measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
					measured                     = true;
				} else if (syncCPU.time_since_epoch().count() % 3 == 0) { // measure slowdown every 3rd step for
					                                                       // an updated estimate
					sim_state_.measured_slowdown = std::chrono::duration<double>(Clock::now() - syncCPU).count() /
					                               Seconds(data_->time - syncSim).count();
				}
			}

			// Call mj_step
			WrappedStep();

			if (num_steps_until_exit_ > 0) {
				num_steps_until_exit_--;
			}

			// Break if reset
			if (data_->time < prevSim) {
				break;
			}
		}
	}
}

} // namespace mujoco_ros
