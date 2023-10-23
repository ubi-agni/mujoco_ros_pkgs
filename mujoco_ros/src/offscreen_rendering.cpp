/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
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

#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/offscreen_camera.h>

#include <sstream>

namespace mujoco_ros {

void MujocoEnv::initializeRenderResources()
{
	image_transport::ImageTransport it(*nh_);
	bool config_exists, use_segid;
	rendering::streamType stream_type;
	std::string cam_name, cam_config_path;
	float pub_freq;
	int max_res_h = 0, max_res_w = 0;

	// TODO(dleins): move camera pub config to URDF/SRDF config once it's ready
	config_exists = nh_->searchParam("cam_config", cam_config_path) || ros::param::search("cam_config", cam_config_path);
	ROS_DEBUG_STREAM_COND(config_exists, "Found camera config under path: " << cam_config_path);

	if (this->model_->ncam == 0) {
		ROS_DEBUG_NAMED("offscreen_rendering", "Model has no cameras, skipping offscreen render utils init");
		return;
	}

	ROS_DEBUG_STREAM("Model has " << this->model_->ncam << " cameras");

	offscreen_.cams.clear();

	rendering::OffscreenCameraPtr cam_ptr;
	int res_h, res_w;
	for (uint8_t cam_id = 0; cam_id < this->model_->ncam; cam_id++) {
		cam_name = mj_id2name(this->model_.get(), mjOBJ_CAMERA, cam_id);
		ROS_DEBUG_STREAM_NAMED("offscreen_rendering",
		                       "Found camera '" << cam_name << "' with id " << cam_id << ". Setting up publishers...");

		std::string param_path(cam_config_path);
		param_path += "/" + cam_name;
		std::string stream_type_string(param_path);
		stream_type_string += "/stream_type";
		std::string pub_freq_string(param_path);
		pub_freq_string += "/frequency";
		std::string segid_string(param_path);
		segid_string += "/use_segid";
		std::string res_w_string(param_path);
		res_w_string += "/width";
		std::string res_h_string(param_path);
		res_h_string += "/height";

		stream_type = rendering::streamType(this->nh_->param<int>(stream_type_string, rendering::streamType::RGB));
		pub_freq    = this->nh_->param<float>(pub_freq_string, 15);
		use_segid   = this->nh_->param<bool>(segid_string, true);
		res_w       = this->nh_->param<int>(res_w_string, 720);
		res_h       = this->nh_->param<int>(res_h_string, 480);

		max_res_h = std::max(res_h, max_res_h);
		max_res_w = std::max(res_w, max_res_w);

		cam_ptr.reset(new rendering::OffscreenCamera(cam_id, cam_name, res_w, res_h, stream_type, use_segid, pub_freq,
		                                             &it, nh_.get(), model_.get(), data_.get(), this));

		offscreen_.cams.push_back(cam_ptr);
	}

	if (model_->vis.global.offheight < max_res_h || model_->vis.global.offwidth < max_res_w) {
		ROS_WARN_STREAM_NAMED("offscreen_rendering", "Model offscreen resolution too small for configured cameras, "
		                                             "updating offscreen resolution to fit cam config ... ("
		                                                 << max_res_w << "x" << max_res_h << ")");
		model_->vis.global.offheight = max_res_h;
		model_->vis.global.offwidth  = max_res_w;
	}

	int buffer_size = max_res_w * max_res_h;
	offscreen_.rgb.reset(new unsigned char[buffer_size * 3], std::default_delete<unsigned char[]>());
	offscreen_.depth.reset(new float[buffer_size], std::default_delete<float[]>());

	ROS_DEBUG_NAMED("offscreen_rendering", "Initializing offscreen rendering utils");

	Glfw().glfwMakeContextCurrent(offscreen_.window.get());
	// Glfw().glfwSetWindowSize(offscreen_.window.get(), max_res_w, max_res_h);
	glfwSetWindowSize(offscreen_.window.get(), max_res_w, max_res_h);

	mjr_makeContext(this->model_.get(), &offscreen_.con, 50);
	ROS_DEBUG_NAMED("offscreen_rendering", "\tApplied model to context");
	mjv_makeScene(this->model_.get(), &offscreen_.scn, Viewer::kMaxGeom);
	mjr_setBuffer(mjFB_OFFSCREEN, &offscreen_.con);
}

void MujocoEnv::offscreenRenderLoop()
{
	is_rendering_running_ = 1;
	Glfw().glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
	Glfw().glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	offscreen_.window.reset(Glfw().glfwCreateWindow(800, 600, "Invisible window", nullptr, nullptr),
	                        [](GLFWwindow *window) {
		                        Glfw().glfwMakeContextCurrent(nullptr);
		                        Glfw().glfwDestroyWindow(window);
	                        });

	if (!offscreen_.window) {
		ROS_ERROR_NAMED("offscreen_rendering", "Failed to create offscreen window");
		return;
	}

	Glfw().glfwMakeContextCurrent(offscreen_.window.get());
	Glfw().glfwSwapInterval(0);

	ROS_DEBUG_NAMED("offscreen_rendering", "Creating offscreen rendering resources ...");
	mjv_defaultCamera(&offscreen_.cam);
	// Set to fixed camera
	offscreen_.cam.type = mjCAMERA_FIXED;
	ROS_DEBUG_NAMED("offscreen_rendering", "\tInitialized camera");
	mjr_defaultContext(&offscreen_.con);
	ROS_DEBUG_NAMED("offscreen_rendering", "\tInitialized context");

	mjv_defaultScene(&offscreen_.scn);
	mjv_makeScene(nullptr, &offscreen_.scn, Viewer::kMaxGeom);

	while (ros::ok() && !settings_.exit_request.load()) {
		{
			// Setup rendering resources if requested
			if (settings_.visual_init_request) {
				initializeRenderResources();
				settings_.visual_init_request = false;
			}

			// Wait for render request
			std::unique_lock<std::mutex> lock(offscreen_.render_mutex);
			// ROS_DEBUG_NAMED("offscreen_rendering", "Waiting for render request");
			offscreen_.cond_render_request.wait(
			    lock, [this] { return offscreen_.request_pending.load() || settings_.visual_init_request.load(); });

			// In case of exit request after waiting for render request
			if (!ros::ok() || settings_.exit_request.load()) {
				break;
			}

			if (settings_.visual_init_request.load()) {
				ROS_DEBUG_NAMED("offscreen_rendering", "Initializing render resources");
				initializeRenderResources();
				settings_.visual_init_request = false;
			}

			for (const auto &cam_ptr : offscreen_.cams) {
				cam_ptr->renderAndPublish(&offscreen_);
				// ROS_DEBUG_STREAM("Done rendering for t=" << cam_ptr->scn_state_.data.time);
			}

			// TODO: Check if camstreams scn_state_->data->time is correct (up to lagging 1 step behind)
			// (Afterwards up to 1/freq seconds behind)

			// Render
			// offscreenRenderEnv();
			offscreen_.request_pending.store(false);
		}
	}
	is_rendering_running_ = 0;
	ROS_DEBUG("Exiting offscreen render loop");
}

} // namespace mujoco_ros
