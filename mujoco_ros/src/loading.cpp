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

#include <filesystem>
#include <limits>

#include <mujoco_ros/rendering/frame_capacity.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/viewer.hpp>

namespace fs = std::filesystem;

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

void MujocoEnv::LoadPlugins(PluginGeneration generation)
{
	MJR_DEBUG("Loading MujocoRosPlugins ...");
	const auto report  = plugin_host_->LoadGeneration(model_.get(), data_.get(), model_generation_, generation);
	plugin_generation_ = report.generation;
	for (const auto &stat : report.statistics) {
		MJR_DEBUG_STREAM("Loading plugin " << stat.type << " took " << stat.load_time << " seconds");
	}
	for (const auto &failure : report.failures) {
		MJR_ERROR_STREAM("Plugin '" << failure.name << "' of type '" << failure.type << "' failed: " << failure.error);
	}
	MJR_DEBUG("Done loading MujocoRosPlugins");
}

void MujocoEnv::CompleteEnvSetup()
{
	LoadInitialJointStates();

	MJR_DEBUG("Resetting noise ...");
	free(ctrlnoise_);
	ctrlnoise_ = static_cast<mjtNum *>(mju_malloc(sizeof(mjtNum) * static_cast<size_t>(model_->nu)));
	mju_zero(ctrlnoise_, model_->nu);

	LoadPlugins(PluginGeneration(plugin_generation_.value() + 1));
	ros_api_->UpdateDynamicParams();
	MJR_DEBUG("Env setup complete");
}

void MujocoEnv::PrepareReload()
{
	is_rendering_running_.store(0);
	MJR_DEBUG("\tResetting collision cbs to default");
	for (const auto func : defaultCollisionFunctions) {
		mjCOLLISIONFUNC[func.geom_type1_][func.geom_type2_] = func.collision_cb_;
	}
	defaultCollisionFunctions.clear();
	custom_collisions_.clear();

	// This phase runs under physics_thread_mutex_. PluginHost quiesces before
	// RenderCore teardown, matching the physics -> PluginHost -> RenderCore order.
	plugin_host_->QuiesceAndDestroy();
	if (render_core_) {
		render_core_->StopAcceptingSnapshots();
		render_core_->RequestCancelRenderTurn();
	}
}

void MujocoEnv::ReconfigureRenderCore()
{
	if (!settings_.render_offscreen || !model_ || model_->ncam == 0) {
		is_rendering_running_.store(0);
		return;
	}
	if (!render_core_) {
		throw std::logic_error("RenderCore backend was not created on the configuration thread");
	}

	auto cameras = InitializeRenderResources();
	std::vector<rendering::CameraDescriptor> descriptors;
	descriptors.reserve(cameras.size());
	int max_width                   = 1;
	int max_height                  = 1;
	std::size_t largest_plane_bytes = 1;
	for (const auto &camera : cameras) {
		const auto descriptor = camera->descriptor();
		descriptors.push_back(descriptor);
		max_width  = std::max(max_width, descriptor.width);
		max_height = std::max(max_height, descriptor.height);
		for (const auto plane :
		     { rendering::PlaneKind::kRgb, rendering::PlaneKind::kDepth, rendering::PlaneKind::kSegmentation }) {
			if (rendering::HasPlane(descriptor.planes, plane)) {
				largest_plane_bytes = std::max(largest_plane_bytes, descriptor.layout(plane).byte_length);
			}
		}
	}
	if (largest_plane_bytes > std::numeric_limits<std::size_t>::max() / 3U) {
		throw std::runtime_error("configured render planes exceed frame boundary capacity range");
	}
	std::vector<rendering::CameraHistoryDepth> histories;
	for (const auto &[registration_id, registration] : camera_publication_transport_.python_consumers) {
		(void)registration_id;
		for (const auto &camera : cameras) {
			if (camera->cam_id_ != registration.camera_id) {
				continue;
			}
			histories.push_back({ camera->descriptor().id, registration.history_depth });
			break;
		}
	}
	const std::size_t required_slots = [&]() {
		try {
			return rendering::ComputeFrameSlotCapacity(descriptors, histories);
		} catch (const rendering::FrameCapacityOverflow &error) {
			throw std::runtime_error(std::string("configured frame slot capacity overflow: ") + error.what());
		}
	}();
	const std::size_t required_bytes = [&]() {
		try {
			return rendering::ComputeFrameByteCapacity(descriptors, histories);
		} catch (const rendering::FrameCapacityOverflow &error) {
			throw std::runtime_error(std::string("configured frame byte capacity overflow: ") + error.what());
		}
	}();
	const auto max_slots = render_core_->frames().max_slots();
	const auto max_bytes = render_core_->frames().max_bytes();
	if (required_slots > max_slots) {
		throw std::runtime_error("configured frame slot capacity " + std::to_string(required_slots) +
		                         " exceeds RenderCore frame boundary budget " + std::to_string(max_slots));
	}
	if (required_bytes > max_bytes) {
		throw std::runtime_error("configured frame byte capacity " + std::to_string(required_bytes) +
		                         " exceeds RenderCore frame boundary byte budget " + std::to_string(max_bytes));
	}
	const auto warm_slot_count =
	    rendering::ComputeWarmSlotCount(max_slots, max_bytes, largest_plane_bytes, required_slots);
	if (warm_slot_count < required_slots) {
		throw std::runtime_error("configured frame storage requires " + std::to_string(required_slots) +
		                         " warmed slots but byte budget allows " + std::to_string(warm_slot_count));
	}
	// Stage the owned model before changing RenderCore or exposing any camera state.
	auto next_render_model_copy = rendering::CopyModel(*model_);
	const auto status = render_core_->Reconfigure(model_generation_, FrameGeneration(frame_generation_.value() + 1),
	                                              rendering::FrameLayout(max_width, max_height, largest_plane_bytes),
	                                              descriptors, warm_slot_count);
	if (!status.ok()) {
		is_rendering_running_.store(0);
		render_core_->StopAcceptingSnapshots();
		throw std::runtime_error("RenderCore reconfiguration failed: " + status.message);
	}
	if (IsShutdownRequested()) {
		render_core_->StopAcceptingSnapshots();
		is_rendering_running_.store(0);
		return;
	}
	// Publish the owned model snapshot before making the new camera generation visible.
	render_model_copy_ = std::move(next_render_model_copy);
	camera_publication_transport_.RecordRenderStatus(rendering::FrameStatus::Ok());
	{
		std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
		is_rendering_running_.store(1);
		for (const auto &camera : cameras) {
			camera->SetActiveRenderCore(render_core_);
			camera->RegisterConsumer(*render_core_);
		}
		frame_generation_                  = render_core_->frames().generation();
		camera_publication_transport_.cams = cameras;
		camera_publication_transport_.RebindPythonConsumersLocked();
	}
}

void MujocoEnv::LoadWithModelAndData()
{
	if (IsShutdownRequested()) {
		throw std::runtime_error("model load aborted because environment shutdown was requested");
	}
	{
		RecursiveLock physics_lock(physics_thread_mutex_);
		CloseRenderTurnAdmission();
		PrepareReload();
		OnReloadPhase(ReloadPhase::kRenderQuiescenceStarted);
		auto render_core = render_core_;
		physics_lock.unlock();
		if (render_core) {
			const auto render_status = render_core->FinishOrCancelRenderTurn();
			if (!render_status.ok()) {
				MJR_WARN_STREAM("RenderCore was unavailable while quiescing for model reload: " << render_status.message);
			}
		}
		// RenderCore completion does not include the caller-side publication and
		// consumer-delivery tail of SubmitRenderSnapshot(). Drain that accepted
		// render turn before replacing cameras, model, or frame generations.
		WaitForInFlightRenderTurns();
		OnReloadPhase(ReloadPhase::kRenderTurnsIdle);
		std::string retirement_errors;
		if (!RetireRenderResources(&retirement_errors)) {
			throw std::runtime_error("camera retirement failed before model replacement: " + retirement_errors);
		}
		physics_lock.lock();
		render_model_copy_.reset();
		OnReloadPhase(ReloadPhase::kOldGenerationQuiesced);

		std::shared_ptr<mjModel> mold;
		std::shared_ptr<mjData> dold;
		if (mnew == nullptr || dnew == nullptr) {
			throw std::runtime_error("model load aborted because no queued model is available");
		}
		if (settings_.is_python_request.load()) {
			mold = std::shared_ptr<mjModel>(mnew, [](mjModel *) {});
			dold = std::shared_ptr<mjData>(dnew, [](mjData *) {});
			settings_.is_python_request.store(0);
		} else {
			mold = std::shared_ptr<mjModel>(mnew, mj_deleteModel);
			dold = std::shared_ptr<mjData>(dnew, mj_deleteData);
		}
		// The shared owners now own the queued pointers; failure cleanup must not delete them again.
		mnew = nullptr;
		dnew = nullptr;

		// Swap the new model and data with the old ones
		std::atomic_store(&model_, mold);
		std::atomic_store(&data_, dold);
		model_generation_ = ModelGeneration(model_generation_.value() + 1);
		OnReloadPhase(ReloadPhase::kModelSwapped);

		// perform a forward pass to initialize all fields if not done yet (very important for offscreen rendering)
		mj_forward(model_.get(), data_.get());
		if (settings_.render_offscreen && model_->ncam > 0) {
			snapshot_pool_.Activate(*model_, model_generation_);
		} else {
			snapshot_pool_.Deactivate();
		}
		OnReloadPhase(ReloadPhase::kForwarded);

		if (has_pending_runtime_options_) {
			const auto pending_patch     = pending_runtime_options_;
			has_pending_runtime_options_ = false;
			pending_runtime_options_     = {};
			const auto candidate         = MergeRuntimeOptions(ReadRuntimeOptions(model_->opt), pending_patch);
			const auto valid             = ValidateRuntimeOptions(candidate);
			if (valid.ok()) {
				mjOption candidate_option = model_->opt;
				WriteRuntimeOptions(candidate, candidate_option);
				model_->opt    = candidate_option;
				options_epoch_ = OptionsEpoch(options_epoch_.value() + 1);
			} else {
				const std::string message =
				    "Runtime Options startup update rejected: " + valid.error->field + ": " + valid.error->message;
				mju::strcpy_arr(load_error_, message.c_str());
			}
		}

		if (model_->opt.integrator == mjINT_EULER) {
			MJR_WARN(
			    "Euler integrator detected. Euler is default for legacy reasons, consider using implicitfast, which is"
			    "recommended for most applications.");
		}

		if (threadpool_ != nullptr) {
			mju_bindThreadPool(data_.get(), threadpool_);
		}

		CompleteEnvSetup();
	}

	// RenderCore may wait for an in-flight render. Keep that wait outside all
	// environment and physics locks. PluginHost setup above remains ordered first.
	OnReloadPhase(ReloadPhase::kRenderReconfigureStarted);
	ReconfigureRenderCore();
	ConnectedViewersLease connected_viewers;
	{
		RecursiveLock physics_lock(physics_thread_mutex_);
		MJR_DEBUG("Delegating model loading to viewers");
		connected_viewers = AcquireConnectedViewersLease();
	}
	for (const auto viewer : connected_viewers.viewers()) {
		try {
			viewer->Load(model_, data_, filename_, model_generation_);
		} catch (const ViewerLoadRejected &) {
			MJR_DEBUG("Viewer stopped before accepting the new model generation");
		}
	}
}

bool MujocoEnv::InitModelFromQueue()
{
	// clear previous error message
	load_error_[0] = '\0';

	bool is_new = false;
	if (queued_filename_[0]) {
		is_new = mju::strcmp_arr(filename_, queued_filename_);
	}

	if (!is_new) {
		mju::strcpy_arr(queued_filename_, filename_);
	}

	bool is_mjb = false;
	if (mju::strlen_arr(queued_filename_) > 4 &&
	    !std::strncmp(queued_filename_ + mju::strlen_arr(queued_filename_) - 4, ".mjb",
	                  mju::sizeof_arr(queued_filename_) - mju::strlen_arr(queued_filename_) + 4)) {
		is_mjb = true;
	}

	bool is_file = false;
	if (mju::strlen_arr(queued_filename_) > 4 &&
	    !std::strncmp(queued_filename_ + mju::strlen_arr(queued_filename_) - 4, ".xml",
	                  mju::sizeof_arr(queued_filename_) - mju::strlen_arr(queued_filename_) + 4)) {
		is_file = true;
	} else {
		try {
			is_file = std::filesystem::is_regular_file(fs::status(queued_filename_));
		} catch (const std::filesystem::filesystem_error &ex) {
			MJR_DEBUG_STREAM("\tFilesystem error while checking for regular file: " << ex.what());
		}
	}

	if (is_file) {
		MJR_DEBUG("\tModel is a regular file. Loading from filesystem");
	} else if (queued_filename_[0] != '\0') { // new model string
		MJR_DEBUG("\tModel is not a regular file. Loading from string");

		MJR_WARN("Loading nested resources (textures, meshes, ...) from string is broken since 2.3.4. A fix is on the "
		         "way (see https://github.com/deepmind/mujoco/discussions/957#discussion-5348269)");

		mj_addBufferVFS(&vfs_, "model_testing", queued_filename_, mju::strlen_arr(queued_filename_));
		MJR_DEBUG("\tSaved string content to VFS");
	}

	auto load_start = Clock::now();
	if (is_mjb) {
		MJR_DEBUG("\tLoading mjb file");
		mnew = mj_loadModel(queued_filename_, nullptr);
	} else {
		if (is_file) {
			MJR_DEBUG("\tLoading xml file");
			mnew = mj_loadXML(queued_filename_, nullptr, load_error_, kErrorLength);
		} else {
			MJR_DEBUG("\tLoading virtual file from VFS");
			mnew = mj_loadXML("model_testing", &vfs_, load_error_, kErrorLength);
		}
	}

	auto load_interval  = Clock::now() - load_start;
	double load_seconds = Seconds(load_interval).count();

	if (!mnew) {
		const auto connected_viewers = AcquireConnectedViewersLease();
		for (const auto viewer : connected_viewers.viewers()) {
			mju::strcpy_arr(viewer->load_error, load_error_);
		}

		MJR_ERROR_STREAM("Loading new model failed: " << load_error_);
		MJR_DEBUG("\tRolling back old model");

		if (!is_file) {
			mj_deleteFileVFS(&vfs_, "model_testing");
		}

		// 'clear' new filename
		queued_filename_[0] = '\0';

		sim_state_.model_valid       = false;
		has_pending_runtime_options_ = false;
		pending_runtime_options_     = {};
		return false;
	}

	MJR_INFO_STREAM("Model loaded in " << load_seconds << " seconds");
	MJR_DEBUG("Model compiled successfully");
	dnew = mj_makeData(mnew);

	if (!is_file) {
		MJR_DEBUG("\tAdding new model permanently to VFS");
		std::size_t length = mju::strlen_arr(queued_filename_);
		mj_addBufferVFS(&vfs_, "model_string", queued_filename_, length);
		mj_deleteFileVFS(&vfs_, "model_testing");
	}

	// 'clear' filename in queue
	mju::strcpy_arr(filename_, queued_filename_);
	queued_filename_[0] = '\0';
	// delete allocated memory for VFS backup

	// Compiler warning: print and pause
	if (load_error_[0]) {
		// next mj_forward will print the message
		MJR_WARN_STREAM("Model compiled, but got simulation warning: " << load_error_);
		if (!IsHeadless())
			SetPaused(true);
	} else if (load_seconds > 0.25) {
		mju::sprintf_arr(load_error_, "Model loaded in %.2g seconds", load_seconds);
	}

	const auto connected_viewers = AcquireConnectedViewersLease();
	for (const auto viewer : connected_viewers.viewers()) {
		mju::strcpy_arr(viewer->load_error, load_error_);
	}

	// Update real-time settings
	int num_clicks  = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
	float min_error = 1e6f;
	float desired   = 1.0f;
#if MJR_ROS_VERSION == ROS_1
	nh_->param<float>("realtime", desired, mnew->vis.global.realtime);
#else // MJR_ROS_VERSION == ROS_2
	this->get_parameter("realtime", desired);
#endif

	if (desired == -1.f) {
		SetRealTimeIndex(0);
	} else if (desired <= 0.f or desired > 1.f) {
		MJR_WARN("Desired realtime should be in range (0, 1]. Falling back to default (1)");
		SetRealTimeIndex(1);
	} else {
		int real_time_index = 1;
		desired             = mju_log(100 * desired);
		for (int click = 0; click < num_clicks; click++) {
			float error = mju_abs(mju_log(percentRealTime[click]) - desired);
			if (error < min_error) {
				min_error       = error;
				real_time_index = click;
			}
		}
		SetRealTimeIndex(real_time_index);
	}

	return true;
}

} // namespace mujoco_ros
