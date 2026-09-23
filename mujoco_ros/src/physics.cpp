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

#include <mujoco/mujoco.h>

#include <stdexcept>

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

void MujocoEnv::SubmitRenderSnapshot(RenderSnapshotBatch batch)
{
	if (!render_core_ || !batch.model || !batch.data || batch.cameras.empty() || batch.plans.empty() ||
	    batch.plans.size() != batch.accepted_publication_sequences.size()) {
		const auto generation = render_core_ ? render_core_->frames().generation() : frame_generation_;
		camera_publication_transport_.RecordRenderStatus(
		    rendering::FrameStatus{ rendering::FrameStatusCode::kTerminalError, 0, std::nullopt, generation,
		                            "invalid render snapshot submission" });
		MJR_ERROR("Rejected invalid render snapshot submission");
		return;
	}
	const auto render_core                     = render_core_;
	const auto cameras                         = std::move(batch.cameras);
	const auto &plans                          = batch.plans;
	const auto &accepted_publication_sequences = batch.accepted_publication_sequences;
	const auto ros_time                        = util::toRosTime(batch.simulation_time.count());

	try {
		auto snapshot = std::make_shared<rendering::RenderSnapshot>();
		batch_snapshot_assembly_count_.fetch_add(1, std::memory_order_relaxed);
		snapshot->model_generation   = batch.model_generation;
		snapshot->simulation_time_ns = batch.simulation_time.count();
		snapshot->model              = batch.model;
		snapshot->data               = batch.data;
		snapshot->plugin_geometry    = batch.plugin_geometry;
		for (std::size_t plan_index = 0; plan_index < plans.size(); ++plan_index) {
			const auto &plan     = plans[plan_index];
			const auto submitted = render_core->SubmitSnapshot(snapshot, plan);
			if (!submitted.ok()) {
				camera_publication_transport_.RecordRenderStatus(submitted);
				throw std::runtime_error("RenderCore rejected an owned render snapshot: " + submitted.message);
			}
			const auto completed = render_core->FinishOrCancelRenderTurn();
			if (!completed.ok()) {
				camera_publication_transport_.RecordRenderStatus(completed);
				WarnFrameSlotDrop(completed);
				if (rendering::IsContextIntegrityFailure(completed)) {
					throw std::runtime_error("RenderCore failed an owned render snapshot: " + completed.message);
				}
			}

			const auto camera_index = static_cast<std::size_t>(plan.camera.id - 1);
			if (camera_index >= cameras.size() || cameras[camera_index]->descriptor().id != plan.camera.id) {
				throw std::runtime_error("RenderCore completed a capture for an unknown configured camera");
			}
			const auto publish_result = cameras[camera_index]->PublishLatest(
			    *render_core, ros_time, accepted_publication_sequences[plan_index], render_core->LastTurnCaptureId());
			if (publish_result.accepted || !publish_result.ok()) {
				if (rendering::ShouldRecordPublicationStatus(completed, publish_result.status)) {
					camera_publication_transport_.RecordRenderStatus(publish_result.status);
				}
			}
			if (!publish_result.ok()) {
				MJR_ERROR_STREAM("ROS camera publication rejected: code="
				                 << static_cast<int>(publish_result.status.code) << ", capture_id="
				                 << publish_result.status.capture_id << ", message=" << publish_result.status.message);
				for (const auto consumer : plan.consumers) {
					if (consumer != cameras[camera_index]->ros_consumer_ || publish_result.accepted) {
						render_core->MarkDelivered(plan, consumer);
					}
				}
				continue;
			}
			for (const auto consumer : plan.consumers) {
				if (consumer == cameras[camera_index]->ros_consumer_ && !publish_result.accepted) {
					continue;
				}
				render_core->MarkDelivered(plan, consumer);
			}
		}
	} catch (const std::exception &error) {
		if (camera_publication_transport_.LastRenderStatus().ok()) {
			camera_publication_transport_.RecordRenderStatus(
			    rendering::FrameStatus{ rendering::FrameStatusCode::kBackendFailure, 0, std::nullopt,
			                            render_core->frames().generation(), error.what() });
		}
		const auto status = render_core->Status();
		MJR_ERROR_STREAM("Offscreen render turn failed: " << error.what()
		                                                  << "; RenderCore status code=" << static_cast<int>(status.code)
		                                                  << ", message=" << status.message);
		return;
	} catch (...) {
		if (camera_publication_transport_.LastRenderStatus().ok()) {
			camera_publication_transport_.RecordRenderStatus(
			    rendering::FrameStatus{ rendering::FrameStatusCode::kBackendFailure, 0, std::nullopt,
			                            render_core->frames().generation(), "unknown offscreen render failure" });
		}
		const auto status = render_core->Status();
		MJR_ERROR_STREAM("Offscreen render turn failed with an unknown exception; RenderCore status code="
		                 << static_cast<int>(status.code) << ", message=" << status.message);
		return;
	}
}

void MujocoEnv::WrappedStep()
{
	if (reload_in_progress_.load(std::memory_order_acquire)) {
		return;
	}
	mj_step(model_.get(), data_.get());
	ros_api_->PublishSimTime(data_->time);
	RunLastStageCbs();
	const char *message = Diverged(model_->opt.disableflags, data_.get());

	if (message) {
		MJR_WARN("Simulation diverged: %s", message);
		const auto connected_viewers = AcquireConnectedViewersLease();
		for (const auto &viewer : connected_viewers.viewers()) {
			mju::strcpy_arr(viewer->load_error, message);
		}
	}

	std::optional<RenderSnapshotBatch> render_batch;
	if (settings_.render_offscreen && render_core_ && model_->ncam > 0) {
		RenderSnapshotBatch batch;
		batch.model            = render_model_copy_;
		batch.model_generation = model_generation_;
		batch.simulation_time  = std::chrono::nanoseconds(util::simTimeToNanoseconds(data_->time));
		{
			std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
			batch.cameras = camera_publication_transport_.cams;
		}
		const auto ros_time = util::toRosTime(batch.simulation_time.count());
		batch.plans.reserve(batch.cameras.size());
		batch.accepted_publication_sequences.reserve(batch.cameras.size());
		for (const auto &camera : batch.cameras) {
			const auto accepted_publication_sequence = camera->UpdateDemand(*render_core_, ros_time);
			auto plan = render_core_->EvaluateDemand(batch.simulation_time, camera->descriptor().id);
			if (!plan.consumers.empty()) {
				batch.plans.push_back(std::move(plan));
				batch.accepted_publication_sequences.push_back(accepted_publication_sequence);
			}
		}
		if (batch.model && !batch.cameras.empty() && !batch.plans.empty()) {
			const auto pooled_data = snapshot_pool_.Acquire(*batch.model, *data_, batch.model_generation);
			if (!pooled_data.ok()) {
				const auto code = pooled_data.code == rendering::SnapshotPool::AcquireCode::kExhausted ?
				                      rendering::FrameStatusCode::kSnapshotPoolExhausted :
				                      rendering::FrameStatusCode::kFrameUnavailable;
				camera_publication_transport_.RecordRenderStatus(
				    rendering::FrameStatus{ code, 0, std::nullopt, frame_generation_, pooled_data.message });
				return;
			}
			batch.data = pooled_data.data;
			mjvScene callback_scene;
			mjv_defaultScene(&callback_scene);
			mjv_makeScene(model_.get(), &callback_scene, 10000);
			render_callback_scene_count_.fetch_add(1, std::memory_order_relaxed);
			RunRenderCbs(&callback_scene);
			auto plugin_geometry = std::make_shared<std::vector<mjvGeom>>();
			plugin_geometry->assign(callback_scene.geoms, callback_scene.geoms + callback_scene.ngeom);
			batch.plugin_geometry = std::move(plugin_geometry);
			mjv_freeScene(&callback_scene);
			render_batch.emplace(std::move(batch));
		}
	}
	if (render_batch) {
		// MuJoCo ownership ends with the owned model/data snapshot. RenderCore and
		// consumer completion must not extend the physics mutex critical section.
		try {
			auto turn = BeginInFlightRenderTurn();
			if (!turn) {
				return;
			}
			physics_thread_mutex_.unlock();
			bool capacity_ready = true;
			for (const auto &plan : render_batch->plans) {
				const auto capacity = render_core_->WaitForFrameCapacity(plan);
				if (!capacity.ok()) {
					camera_publication_transport_.RecordRenderStatus(capacity);
					capacity_ready = false;
					break;
				}
			}
			if (capacity_ready) {
				SubmitRenderSnapshot(std::move(*render_batch));
			}
		} catch (...) {
			physics_thread_mutex_.lock();
			throw;
		}
		physics_thread_mutex_.lock();
	}
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
		std::function<void()> physics_pre_lock_hook;
		{
			std::lock_guard<std::mutex> lock(physics_test_hook_mutex_);
			physics_pre_lock_hook = physics_pre_lock_test_hook_;
		}
		if (physics_pre_lock_hook) {
			physics_pre_lock_hook();
		}
		if (control_state_.Snapshot().reset_requested || reset_in_progress_.load(std::memory_order_acquire))
			continue;
		if (reload_in_progress_.load(std::memory_order_acquire))
			continue;

		// Try acquiring the sim mutex
		if (!physics_thread_mutex_.try_lock()) {
			// If mutex is locked, try again later
			continue;
		}

		const auto locked_control_snapshot = control_state_.Snapshot();
		const auto locked_model            = std::atomic_load(&model_);
		const auto locked_data             = std::atomic_load(&data_);
		if (!locked_model || !locked_data || locked_control_snapshot.reset_requested ||
		    reset_in_progress_.load(std::memory_order_acquire) || reload_in_progress_.load(std::memory_order_acquire)) {
			physics_thread_mutex_.unlock();
			continue;
		}

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
	MJR_DEBUG("Exiting physics loop");
}

void MujocoEnv::SimPausedPhysics(mjtNum &syncSim)
{
	const auto startCPU = Clock::now();
	if (HasManualStepRequest()) { // Action call or arrow keys used for stepping
		syncSim = data_->time;
		// Headless / no interactive viewer: drain the full manual-step batch per physics-loop
		// visit. With a connected viewer, time-slice so Sync/render can take the physics mutex.
		const bool time_slice_for_viewer = HasConnectedViewers();
		const auto slice_limit           = Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_);

		while (HasManualStepRequest()) {
			if (time_slice_for_viewer && Clock::now() - startCPU >= slice_limit) {
				break;
			}
			WrappedStep();
			RecordCompletedManualStep();
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

	const auto speed_settings = ConsumeSpeedSettingsSnapshot();

	// Requested slow-down factor
	double slowdown = 100 / percentRealTime[speed_settings.real_time_index];

	// Misalignment condition: distance from target sim time is bigger than syncsimalign
	bool misaligned    = std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;
	bool speed_changed = speed_settings.speed_changed;

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
		// possible. Time-slice only when an interactive viewer needs periodic mutex access.
		const bool time_slice_for_viewer = HasConnectedViewers();
		const auto slice_limit           = Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_);
		while ((speed_settings.real_time_index == 0 ||
		        Seconds((data_->time - syncSim) * slowdown) < Clock::now() - syncCPU) &&
		       (!time_slice_for_viewer || Clock::now() - startCPU < slice_limit) && !IsShutdownRequested() &&
		       num_steps_until_exit_ != 0 && control_state_.Snapshot().running) {
			// measure slowdown before first step
			if (!measured && elapsedSim) {
				if (speed_settings.real_time_index != 0) {
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
