/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
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

#include <filesystem>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/viewer.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#else // MJR_ROS_VERSION == ROS_2
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#endif

namespace fs = std::filesystem;

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

void MujocoEnv::LoadPlugins()
{
	MJR_DEBUG("Loading MujocoRosPlugins ...");
	cb_ready_plugins_.clear();
	cb_ready_plugins_.shrink_to_fit();

#if MJR_ROS_VERSION == ROS_1
	XmlRpc::XmlRpcValue plugin_config;
	if (plugin_utils::ParsePlugins(nh_.get(), plugin_config)) {
		plugin_utils::RegisterPlugins(nh_->getNamespace(), plugin_config, plugins_, this);
	}
#else // MJR_ROS_VERSION == ROS_2
	std::vector<std::string> plugin_names;
	if (plugin_utils::ParsePlugins(this, plugin_names)) {
		plugin_utils::RegisterPlugins(plugin_names, plugins_, this);
	}
#endif

	for (const auto &plugin : plugins_) {
		if (plugin->SafeLoad(model_.get(), data_.get())) {
			cb_ready_plugins_.emplace_back(plugin.get());
		}
		MJR_DEBUG_STREAM("Loading plugin " << plugin->get_type() << " took " << plugin->get_load_time() << " seconds");
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

	LoadPlugins();
	// 	updateDynamicParams();
	MJR_DEBUG("Env setup complete");
}

void MujocoEnv::PrepareReload()
{
	MJR_DEBUG("\tResetting collision cbs to default");
	for (const auto func : defaultCollisionFunctions) {
		mjCOLLISIONFUNC[func.geom_type1_][func.geom_type2_] = func.collision_cb_;
	}
	defaultCollisionFunctions.clear();
	custom_collisions_.clear();

	offscreen_.rgb.reset();
	offscreen_.depth.reset();
	cb_ready_plugins_.clear();
	plugins_.clear();
	offscreen_.cams.clear();
}

void MujocoEnv::LoadWithModelAndData()
{
	{
		// Wrap the new model and data in shared pointers
		std::shared_ptr<mjModel> mold(mnew, mj_deleteModel);
		std::shared_ptr<mjData> dold(dnew, mj_deleteData);

		// Swap the new model and data with the old ones
		std::atomic_store(&model_, mold);
		std::atomic_store(&data_, dold);
	}

	// perform a forward pass to initialize all fields if not done yet (very important for offscreen rendering)
	mj_forward(model_.get(), data_.get());

	if (model_->opt.integrator == mjINT_EULER) {
		MJR_WARN("Euler integrator detected. Euler is default for legacy reasons, consider using implicitfast, which is"
		         "recommended for most applications.");
	}

	if (threadpool_ != nullptr) {
		mju_bindThreadPool(data_.get(), threadpool_);
	}

	PrepareReload();
	CompleteEnvSetup();

	MJR_DEBUG("Delegating model loading to viewers");
	for (const auto viewer : connected_viewers_) {
		viewer->Load(model_, data_, filename_);
	}

	if (settings_.render_offscreen) {
		MJR_DEBUG("Issuing (re)initialization of offscreen rendering resources");
		settings_.visual_init_request.store(1);
		while (settings_.visual_init_request.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			offscreen_.cond_render_request.notify_one();
		}
	}

	load_error_[0]         = '\0';
	sim_state_.model_valid = true;
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
		for (const auto viewer : connected_viewers_) {
			mju::strcpy_arr(viewer->load_error, load_error_);
		}

		MJR_ERROR_STREAM("Loading new model failed: " << load_error_);
		MJR_DEBUG("\tRolling back old model");

		if (!is_file) {
			mj_deleteFileVFS(&vfs_, "model_testing");
		}

		// 'clear' new filename
		queued_filename_[0] = '\0';

		sim_state_.model_valid = false;
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
		if (!settings_.headless)
			settings_.run = 0;
	} else {
		if (load_seconds > 0.25) {
			mju::sprintf_arr(load_error_, "Model loaded in %.2g seconds", load_seconds);
		}
	}

	for (const auto viewer : connected_viewers_) {
		mju::strcpy_arr(viewer->load_error, load_error_);
	}

	// Update real-time settings
	int num_clicks  = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
	float min_error = 1e6f;
	float desired   = 0.f;
#if MJR_ROS_VERSION == ROS_1
	nh_->param<float>("realtime", desired, mnew->vis.global.realtime);
#else // MJR_ROS_VERSION == ROS_2
	this->get_parameter("realtime", desired);
#endif

	if (desired == -1.f) {
		settings_.real_time_index = 0;
	} else if (desired <= 0.f or desired > 1.f) {
		MJR_WARN("Desired realtime should be in range (0, 1]. Falling back to default (1)");
		settings_.real_time_index = 1;
	} else {
		desired = mju_log(100 * desired);
		for (int click = 0; click < num_clicks; click++) {
			float error = mju_abs(mju_log(percentRealTime[click]) - desired);
			if (error < min_error) {
				min_error                 = error;
				settings_.real_time_index = click;
			}
		}
	}

	return true;
}

} // namespace mujoco_ros
