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

#if RENDER_BACKEND == EGL_BACKEND
#include <EGL/egl.h>
#include <EGL/eglext.h>

static constexpr int MAX_EGL_DEVICES = 6;

bool get_egl_device(EGLDeviceEXT *egl_devices, int &choose_device)
{
	EGLint num_devices;

	// Get devices
	PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
	    reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
	if (eglQueryDevicesEXT(MAX_EGL_DEVICES, egl_devices, &num_devices) != EGL_TRUE) {
		ROS_ERROR_STREAM("Failed to query EGL devices. Error type: " << eglGetError());
		return false;
	}
	ROS_DEBUG_STREAM("Found " << num_devices << " EGL devices");

	PFNEGLQUERYDEVICESTRINGEXTPROC eglQueryDeviceStringEXT =
	    reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(eglGetProcAddress("eglQueryDeviceStringEXT"));
	const char *extensions;
	choose_device = 0;
	for (int i = 0; i < num_devices; i++) {
		extensions = eglQueryDeviceStringEXT(egl_devices[i], EGL_EXTENSIONS);
		ROS_DEBUG_STREAM("Device " << i << " has extensions: " << extensions);
		if (strstr(extensions, "EGL_NV_device_cuda")) {
			ROS_DEBUG_STREAM("Choosing device " << i << " for CUDA support");
			choose_device = i;
			break;
		}
	}
	return true;
}

#endif

namespace mujoco_ros {

OffscreenRenderContext::~OffscreenRenderContext()
{
#if RENDER_BACKEND == GLFW_BACKEND
	if (window != nullptr) {
		ROS_DEBUG("Freeing GLFW offscreen context");
		std::unique_lock<std::mutex> lock(render_mutex);
		request_pending.store(false);
		mjv_freeScene(&scn);
		mjr_defaultContext(&con);
		mjr_freeContext(&con);
	}
#elif RENDER_BACKEND == EGL_BACKEND
	ROS_DEBUG("Freeing EGL offscreen context");
	mjv_freeScene(&scn);
	mjr_defaultContext(&con);
	mjr_freeContext(&con);

	EGLDeviceEXT egl_devices[MAX_EGL_DEVICES];
	int choosen_device = 0;
	if (!get_egl_device(egl_devices, choosen_device)) {
		return;
	}

	EGLDisplay display = eglGetPlatformDisplay(EGL_PLATFORM_DEVICE_EXT, egl_devices[choosen_device], nullptr);
	if (display != EGL_NO_DISPLAY) {
		// Get current context
		EGLContext current_context = eglGetCurrentContext();

		// Release context
		eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

		// Destroy context if valid
		if (current_context != EGL_NO_CONTEXT) {
			eglDestroyContext(display, current_context);
		}

		// Terminate display
		eglTerminate(display);
	}
#elif RENDER_BACKEND == OSMESA_BACKEND
	if (!osmesa.initialized) {
		return;
	}
	ROS_DEBUG("Freeing OSMesa offscreen context");
	mjv_freeScene(&scn);
	mjr_defaultContext(&con);
	mjr_freeContext(&con);
	OSMesaDestroyContext(osmesa.ctx);
#endif
}

void MujocoEnv::initializeRenderResources()
{
	bool config_exists, use_segid;
	rendering::streamType stream_type;
	std::string cam_name, cam_config_path, base_topic, rgb, depth, segment;
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
		std::string base_topic_string(param_path);
		base_topic_string += "/topic";
		std::string rgb_topic_string(param_path);
		rgb_topic_string += "/name_rgb";
		std::string depth_topic_string(param_path);
		depth_topic_string += "/name_depth";
		std::string segment_topic_string(param_path);
		segment_topic_string += "/name_segment";

		stream_type = rendering::streamType(this->nh_->param<int>(stream_type_string, rendering::streamType::RGB));
		pub_freq    = this->nh_->param<float>(pub_freq_string, 15);
		use_segid   = this->nh_->param<bool>(segid_string, true);
		res_w       = this->nh_->param<int>(res_w_string, 720);
		res_h       = this->nh_->param<int>(res_h_string, 480);
		base_topic  = this->nh_->param<std::string>(base_topic_string, "cameras/" + cam_name);
		rgb         = this->nh_->param<std::string>(rgb_topic_string, "rgb");
		depth       = this->nh_->param<std::string>(depth_topic_string, "depth");
		segment     = this->nh_->param<std::string>(segment_topic_string, "segmented");

		max_res_h = std::max(res_h, max_res_h);
		max_res_w = std::max(res_w, max_res_w);

		offscreen_.cams.emplace_back(std::make_unique<rendering::OffscreenCamera>(
		    cam_id, base_topic, rgb, depth, segment, cam_name, res_w, res_h, stream_type, use_segid, pub_freq, *nh_,
		    model_.get(), data_.get(), this));
	}

	if (model_->vis.global.offheight < max_res_h || model_->vis.global.offwidth < max_res_w) {
		ROS_WARN_STREAM_NAMED("offscreen_rendering", "Model offscreen resolution too small for configured cameras, "
		                                             "updating offscreen resolution to fit cam config ... ("
		                                                 << max_res_w << "x" << max_res_h << ")");
		model_->vis.global.offheight = max_res_h;
		model_->vis.global.offwidth  = max_res_w;
	}

	int buffer_size  = max_res_w * max_res_h;
	offscreen_.rgb   = std::make_unique<unsigned char[]>(buffer_size * 3);
	offscreen_.depth = std::make_unique<float[]>(buffer_size);

	ROS_DEBUG_NAMED("offscreen_rendering", "Initializing offscreen rendering utils");

#if RENDER_BACKEND == GLFW_BACKEND
	Glfw().glfwMakeContextCurrent(offscreen_.window.get());
	// Glfw().glfwSetWindowSize(offscreen_.window.get(), max_res_w, max_res_h);
	glfwSetWindowSize(offscreen_.window.get(), max_res_w, max_res_h);
#endif

	mjr_makeContext(this->model_.get(), &offscreen_.con, 50);
	ROS_DEBUG_NAMED("offscreen_rendering", "\tApplied model to context");
	mjv_makeScene(this->model_.get(), &offscreen_.scn, Viewer::kMaxGeom);
	mjr_setBuffer(mjFB_OFFSCREEN, &offscreen_.con);
}

#if RENDER_BACKEND == EGL_BACKEND
bool MujocoEnv::InitGL()
{
	ROS_DEBUG("Initializing EGL...");

	EGLDeviceEXT egl_devices[MAX_EGL_DEVICES];
	int choosen_device = 0;
	if (!get_egl_device(egl_devices, choosen_device)) {
		return false;
	}

	// clang-format off
	const EGLint config[] = {
		EGL_RED_SIZE,		   8,
		EGL_GREEN_SIZE,		   8,
		EGL_BLUE_SIZE,		   8,
		EGL_ALPHA_SIZE,		   8,
		EGL_DEPTH_SIZE,		   24,
		EGL_STENCIL_SIZE,	   8,
		EGL_COLOR_BUFFER_TYPE, EGL_RGB_BUFFER,
		EGL_SURFACE_TYPE, 	   EGL_PBUFFER_BIT,
		EGL_RENDERABLE_TYPE,   EGL_OPENGL_BIT,
		EGL_NONE };
	// clang-format on

	// Get Display
	EGLDisplay display = eglGetPlatformDisplay(EGL_PLATFORM_DEVICE_EXT, egl_devices[choosen_device], nullptr);
	if (display == EGL_NO_DISPLAY) {
		ROS_ERROR_STREAM("Failed to get EGL display. Error type: " << eglGetError());
		return false;
	}

	// Initialize EGL
	EGLint major, minor;
	if (eglInitialize(display, &major, &minor) != EGL_TRUE) {
		ROS_ERROR_STREAM("Failed to initialize EGL. Error type: " << eglGetError());
		return false;
	}

	// Choose Config
	EGLint num_configs;
	EGLConfig egl_config;
	if (eglChooseConfig(display, config, &egl_config, 1, &num_configs) != EGL_TRUE) {
		ROS_ERROR_STREAM("Failed to choose EGL config. Error type: " << eglGetError());
		return false;
	}

	// bind OpenGL API
	if (eglBindAPI(EGL_OPENGL_API) != EGL_TRUE) {
		ROS_ERROR_STREAM("Failed to bind OpenGL API. Error type: " << eglGetError());
		return false;
	}

	// Create context
	EGLContext context = eglCreateContext(display, egl_config, EGL_NO_CONTEXT, nullptr);
	if (context == EGL_NO_CONTEXT) {
		ROS_ERROR_STREAM("Failed to create EGL context. Error type: " << eglGetError());
		return false;
	}

	// Make current
	if (eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context) != EGL_TRUE) {
		ROS_ERROR_STREAM("Failed to make EGL context current. Error type: " << eglGetError());
		return false;
	}

	ROS_DEBUG("EGL initialized");
	return true;
}
#elif RENDER_BACKEND == OSMESA_BACKEND
bool MujocoEnv::InitGL()
{
	ROS_DEBUG("Initializing OSMesa...");
	// Initialize OSMesa
	offscreen_.osmesa.ctx = OSMesaCreateContextExt(GL_RGBA, 24, 8, 8, nullptr);
	if (!offscreen_.osmesa.ctx) {
		ROS_ERROR("OSMesa context creation failed");
		return false;
	}

	// Make current
	// if (!OSMesaMakeCurrent(offscreen_.osmesa.ctx, offscreen_.osmesa.buffer, GL_UNSIGNED_BYTE, width, height)) {
	if (!OSMesaMakeCurrent(offscreen_.osmesa.ctx, offscreen_.osmesa.buffer, GL_UNSIGNED_BYTE, 800, 800)) {
		ROS_ERROR("OSMesa make current failed");
		return false;
	}
	ROS_DEBUG("OSMesa initialized");
	offscreen_.osmesa.initialized = true;
	return true;
} // InitGL
#endif

void MujocoEnv::offscreenRenderLoop()
{
#if RENDER_BACKEND == GLFW_BACKEND
	Glfw().glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
	Glfw().glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	offscreen_.window.reset(Glfw().glfwCreateWindow(800, 600, "Invisible window", nullptr, nullptr),
	                        [](GLFWwindow *window) {
		                        Glfw().glfwMakeContextCurrent(nullptr);
		                        Glfw().glfwDestroyWindow(window);
	                        });

	if (!offscreen_.window) {
		ROS_ERROR_NAMED("offscreen_rendering", "Failed to create offscreen window");
		settings_.render_offscreen    = false;
		settings_.visual_init_request = false;
		offscreen_.request_pending.store(false);
		return;
	}

	Glfw().glfwMakeContextCurrent(offscreen_.window.get());
	Glfw().glfwSwapInterval(0);
#elif RENDER_BACKEND == EGL_BACKEND
	if (!InitGL()) {
		ROS_ERROR("Failed to initialize EGL. Cannot run offscreen rendering");
		settings_.render_offscreen = false;
		settings_.visual_init_request = false;
		offscreen_.request_pending.store(false);
		return;
	}
#elif RENDER_BACKEND == OSMESA_BACKEND
	if (!InitGL()) {
		ROS_ERROR("Failed to initialize OSMesa. Cannot run offscreen rendering");
		settings_.render_offscreen    = false;
		settings_.visual_init_request = false;
		offscreen_.request_pending.store(false);
		return;
	}
#endif

#if RENDER_BACKEND == NO_BACKEND
	ROS_ERROR("No offscreen rendering backend available. Cannot run offscreen rendering");
	settings_.render_offscreen    = false;
	settings_.visual_init_request = false;
	offscreen_.request_pending.store(false);
	is_rendering_running_ = 0;
	ROS_DEBUG("Exiting offscreen render loop");
	return;
#else
	is_rendering_running_ = 1;
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
			offscreen_.cond_render_request.wait(lock, [this] {
				return offscreen_.request_pending.load() || settings_.visual_init_request.load() ||
				       settings_.exit_request.load();
			});

			// In case of exit request after waiting for render request
			if (!ros::ok() || settings_.exit_request.load()) {
				break;
			}

			if (settings_.visual_init_request.load()) {
				ROS_DEBUG_NAMED("offscreen_rendering", "Initializing render resources");
				initializeRenderResources();
				settings_.visual_init_request = false;
				continue;
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
#endif
}

} // namespace mujoco_ros
