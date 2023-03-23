/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: David P. Leins */

#include <mujoco_ros/rendering/utils.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/glfw_dispatch.h>
#include <mujoco_ros/uitools.h>
#include <mujoco_ros/array_safety.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <fstream>

using namespace MujocoSim::detail;

namespace MujocoSim::rendering {

// Definition of externally declared perturbation
mjvPerturb pert_;

namespace {
void render(GLFWwindow *window)
{
	std::lock_guard<std::mutex> lk(render_mtx);
	glfwMakeContextCurrent(main_window_);
	mjr_setBuffer(mjFB_WINDOW, &free_context_);

	if (!main_env_) {
		ROS_WARN_NAMED("mujoco_render", "no main env!");
		return;
	}

	// get 3D rectangle and reduced for profiler
	mjrRect rect      = uistate_.rect[3];
	mjrRect smallrect = rect;

	if (settings_.profiler) {
		smallrect.width = rect.width - rect.width / 4;
	}

	// no model
	if (!main_env_->model) {
		// blank screen
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

		// label
		if (settings_.loadrequest.load())
			mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, "loading", nullptr, &free_context_);

		//// We don't want this. A model should be loaded over services or during start
		// else
		// mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &(env->vis.con));

		// render uis
		if (settings_.ui0)
			mjui_render(&ui0_, &uistate_, &free_context_);
		if (settings_.ui1)
			mjui_render(&ui1_, &uistate_, &free_context_);

		// finalize
		glfwSwapBuffers(window);
	} else {
		renderCallback(main_env_->data.get(), &free_scene_);
	}
	// render scene
	mjr_render(rect, &free_scene_, &free_context_);

	// show pause/loading label
	std::string pauseloadlabel;
	if (!settings_.run.load() || settings_.loadrequest.load())
		pauseloadlabel = settings_.loadrequest.load() ? "loading" : "pause";

	float desired_realtime = percentRealTime[settings_.rt_index];
	float actual_realtime  = 100 / settings_.measured_slow_down;

	float rt_offset = mju_abs(actual_realtime - desired_realtime);
	bool misaligned = settings_.run.load() && rt_offset > 0.1 * desired_realtime;

	// show realtime label
	char rt_label[30] = { '\0' };
	if (desired_realtime != 100. || misaligned) {
		int labelsize;
		if (desired_realtime != -1) {
			labelsize = std::snprintf(rt_label, sizeof(rt_label), "%g%%", desired_realtime);
		} else {
			labelsize   = 1;
			rt_label[0] = '+';
		}
		std::snprintf(rt_label + labelsize, sizeof(rt_label) - labelsize, " (%-4.1f%%)", actual_realtime);
	}

	if (!pauseloadlabel.empty() || rt_label[0]) {
		std::string newline      = !pauseloadlabel.empty() && rt_label[0] ? "\n" : "";
		std::string topleftlabel = rt_label + newline + pauseloadlabel;
		mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, smallrect, topleftlabel.c_str(), nullptr, &free_context_);
	}

	// show ui 0
	if (settings_.ui0)
		mjui_render(&ui0_, &uistate_, &free_context_);

	// show ui 1
	if (settings_.ui1)
		mjui_render(&ui1_, &uistate_, &free_context_);

	// show help
	if (settings_.help)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title, help_content, &free_context_);

	// show info
	if (settings_.info)
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, info_title, info_content, &free_context_);

	// show profiler
	if (settings_.profiler)
		profilerShow(main_env_, rect);

	// show sensor
	if (settings_.sensor)
		sensorShow(main_env_, smallrect);

	// finalize
	glfwSwapBuffers(window);
}

void initVisible()
{
	// multisampling
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_VISIBLE, 1);
	glfwWindowHint(GLFW_DOUBLEBUFFER, 1);

	// get videomode and save
	vmode_ = *glfwGetVideoMode(glfwGetPrimaryMonitor());

	// create window
	main_window_ = glfwCreateWindow((2 * vmode_.width) / 3, (2 * vmode_.height) / 3, "Simulate", nullptr, nullptr);

	if (!main_window_) {
		glfwTerminate();
		ROS_ERROR_NAMED("mujoco", "Could not create window");
		mju_error("could not create window");
	}

	ROS_DEBUG_NAMED("mujoco_render", "Created main window");

	// save window position and size
	glfwGetWindowPos(main_window_, windowpos_, windowpos_ + 1);
	glfwGetWindowSize(main_window_, windowsize_, windowsize_ + 1);

	// select default font
	int fontscale  = uiFontScale(main_window_);
	settings_.font = fontscale / 50 - 1;

	// make context current, set v-sync
	glfwMakeContextCurrent(main_window_);
	glfwSwapInterval(settings_.vsync);

	mjv_defaultScene(&free_scene_);
	mjr_defaultContext(&free_context_);
	mjv_defaultCamera(&free_camera_);
	mjv_defaultOption(&vopt_);

	// Create scene and context
	mjv_makeScene(nullptr, &free_scene_, rendering::maxgeom_);
	mjr_makeContext(nullptr, &free_context_, 50 * (settings_.font + 1));

	// set GLFW callbacks
	uiSetCallback(main_window_, &uistate_, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(main_window_, render);

	// init state und uis
	std::memset(&uistate_, 0, sizeof(mjuiState));
	std::memset(&ui0_, 0, sizeof(mjUI));
	std::memset(&ui1_, 0, sizeof(mjUI));
	uistate_.userdata = &main_env_;
	ui0_.spacing      = mjui_themeSpacing(settings_.spacing);
	ui0_.color        = mjui_themeColor(settings_.color);
	ui0_.predicate    = uiPredicate;
	ui0_.userdata     = &main_env_;
	ui0_.rectid       = 1;
	ui0_.auxid        = 0;

	ui1_.spacing   = mjui_themeSpacing(settings_.spacing);
	ui1_.color     = mjui_themeColor(settings_.color);
	ui1_.predicate = uiPredicate;
	ui1_.userdata  = &main_env_;
	ui1_.rectid    = 2;
	ui1_.auxid     = 1;

	// populate uis with standard sections
	mjui_add(&ui0_, defFile);
	mjui_add(&ui0_, defOption);
	mjui_add(&ui0_, defSimulation);
	mjui_add(&ui0_, defWatch);
	uiModify(main_window_, &ui0_, &uistate_, &free_context_);
	uiModify(main_window_, &ui1_, &uistate_, &free_context_);
}

bool renderAndPubEnv(MujocoEnvPtr env, const bool rgb, const bool depth, const image_transport::Publisher &pub_rgb,
                     const image_transport::Publisher &pub_depth, const int width, const int height,
                     const std::string cam_name)
{
	if ((!rgb && !depth) || // nothing to render
	    (pub_rgb.getNumSubscribers() == 0 && pub_depth.getNumSubscribers() == 0) || // no subscribers
	    (!depth && pub_rgb.getNumSubscribers() == 0) || // would only render rgb, but has no subscribers
	    (!rgb && pub_depth.getNumSubscribers() == 0)) { // would only render depth, but has no subscribers
		return false;
	}

	// Resize according to camera resolution
	env->vis.con.offWidth  = width;
	env->vis.con.offHeight = height;
	mjrRect viewport       = mjr_maxViewport(&(env->vis.con));

	// Update scene
	mjv_updateScene(env->model.get(), env->data.get(), &(env->vis.vopt), nullptr, &(env->vis.cam), mjCAT_ALL,
	                &(env->vis.scn));
	// render to buffer
	mjr_render(viewport, &env->vis.scn, &env->vis.con);
	// read buffers
	if (rgb && depth) {
		mjr_readPixels(env->vis.rgb.get(), env->vis.depth.get(), viewport, &env->vis.con);
	} else if (rgb) {
		mjr_readPixels(env->vis.rgb.get(), nullptr, viewport, &env->vis.con);
	} else if (depth) {
		mjr_readPixels(nullptr, env->vis.depth.get(), viewport, &env->vis.con);
	}
	glfwSwapBuffers(env->vis.window);

	if (rgb) {
		sensor_msgs::ImagePtr rgb_im = boost::make_shared<sensor_msgs::Image>();
		rgb_im->header.frame_id      = cam_name + "_optical_frame";
		rgb_im->header.stamp         = ros::Time::now();
		rgb_im->width                = viewport.width;
		rgb_im->height               = viewport.height;
		rgb_im->encoding             = sensor_msgs::image_encodings::RGB8;
		rgb_im->step                 = viewport.width * 3 * sizeof(unsigned char);
		size_t size                  = rgb_im->step * viewport.height;
		rgb_im->data.resize(size);

		memcpy((char *)(&rgb_im->data[0]), env->vis.rgb.get(), size);

		for (uint r = 0; r < rgb_im->height / 2; ++r) {
			unsigned char *top_row    = &rgb_im->data[3 * rgb_im->width * r];
			unsigned char *bottom_row = &rgb_im->data[3 * rgb_im->width * (rgb_im->height - 1 - r)];
			std::swap_ranges(top_row, top_row + 3 * rgb_im->width, bottom_row);
		}

		pub_rgb.publish(rgb_im);
	}

	if (depth) {
		sensor_msgs::ImagePtr depth_im = boost::make_shared<sensor_msgs::Image>();
		depth_im->header.frame_id      = cam_name + "_optical_frame";
		depth_im->header.stamp         = ros::Time::now();
		depth_im->width                = viewport.width;
		depth_im->height               = viewport.height;
		depth_im->encoding             = sensor_msgs::image_encodings::TYPE_32FC1;
		depth_im->step                 = viewport.width * sizeof(float);
		size_t size                    = depth_im->step * viewport.height;
		depth_im->data.resize(size);

		uint16_t *dest_uint = (uint16_t *)(&depth_im->data[0]);
		float *dest_float   = (float *)(&depth_im->data[0]);
		uint64_t index      = 0;

		float e = env->model->stat.extent;
		float f = e * env->model->vis.map.zfar;
		float n = e * env->model->vis.map.znear;

		for (int j = depth_im->height - 1; j >= 0; --j) {
			for (uint32_t i = 0; i < depth_im->width; i++) {
				float depth                         = env->vis.depth[index++];
				dest_float[i + j * depth_im->width] = -f * n / (depth * (f - n) - f);
			}
		}

		pub_depth.publish(depth_im);
	}
	return true;
}
} // end unnamed namespace

void renderCallback(mjData *data, mjvScene *scene)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runRenderCbs(scene);
	}
}

bool isWindowClosing()
{
	return glfwWindowShouldClose(main_window_);
}

void deinitVisual()
{
	if (!settings_.headless) {
		uiClearCallback(main_window_);
		mjv_freeScene(&free_scene_);
		mjr_freeContext(&free_context_);
		glfwDestroyWindow(main_window_);
	}
	main_env_.reset();
}

void initVisual()
{
	if (!glfwInit()) {
		ROS_ERROR_NAMED("mujoco", "Could not initialize GLFW");
		const char *error[30];
		glfwGetError(error);
		ROS_DEBUG_STREAM_NAMED("mujoco", "Detailed GLFW error message: " << *error);
		mju_error("Could not initialize GLFW");
	}
	ROS_DEBUG_NAMED("render", "glfwInit successful");
	mjcb_time = timer;

	profilerInit();
	sensorInit();

	if (!settings_.headless) {
		initVisible();
	}
}

void offScreenRenderEnv(MujocoEnvPtr env)
{
	if (!env->vis.window || !settings_.render_offscreen) {
		return;
	}

	glfwMakeContextCurrent(env->vis.window);
	mjr_setBuffer(mjFB_OFFSCREEN, &env->vis.con);

	int type_backup        = env->vis.cam.type;
	int cam_id_backup      = env->vis.cam.fixedcamid;
	mjtByte segment_backup = env->vis.scn.flags[mjRND_SEGMENT];
	mjtByte segid_backup   = env->vis.scn.flags[mjRND_IDCOLOR];

	for (auto &stream : env->cam_streams) {
		env->vis.cam.type       = mjCAMERA_FIXED;
		env->vis.cam.fixedcamid = stream->cam_id;

		if (ros::Duration(1 / stream->pub_freq) >= ros::Time::now() - stream->last_pub) {
			continue;
		}

		int cam_id = stream->cam_id;

		stream->last_pub = ros::Time::now();

		bool rendered = false;

		if (stream->stream_type & streamType::RGB && stream->stream_type & streamType::DEPTH) {
			// RGB and DEPTH
			env->vis.scn.flags[mjRND_SEGMENT] = 0;
			rendered =
			    renderAndPubEnv(env, (stream->stream_type & streamType::RGB), (stream->stream_type & streamType::DEPTH),
			                    stream->rgb_pub, stream->depth_pub, stream->width, stream->height, stream->cam_name);

			if (stream->stream_type & streamType::SEGMENTED) {
				// SEGMENTED additional to RGB and DEPTH
				env->vis.scn.flags[mjRND_IDCOLOR] = stream->use_segid;
				env->vis.scn.flags[mjRND_SEGMENT] = 1;
				rendered = renderAndPubEnv(env, true, false, stream->segment_pub, stream->depth_pub, stream->width,
				                           stream->height, stream->cam_name);
			}

		} else if (stream->stream_type & streamType::SEGMENTED && stream->stream_type & streamType::DEPTH) {
			// SEGMENTED and DEPTH
			env->vis.scn.flags[mjRND_IDCOLOR] = stream->use_segid;
			env->vis.scn.flags[mjRND_SEGMENT] = 1;
			rendered = renderAndPubEnv(env, true, true, stream->segment_pub, stream->depth_pub, stream->width,
			                           stream->height, stream->cam_name);
		} else if (stream->stream_type & streamType::RGB && stream->stream_type & streamType::SEGMENTED, stream->width,
		           stream->height) {
			// RGB and SEGMENTED (needs two calls because both go into the rgb buffer)
			env->vis.scn.flags[mjRND_SEGMENT] = 0;
			rendered = renderAndPubEnv(env, true, false, stream->rgb_pub, stream->depth_pub, stream->width, stream->height,
			                           stream->cam_name);

			env->vis.scn.flags[mjRND_IDCOLOR] = stream->use_segid;
			env->vis.scn.flags[mjRND_SEGMENT] = 1;
			rendered = rendered || renderAndPubEnv(env, true, false, stream->segment_pub, stream->depth_pub, stream->width,
			                                       stream->height, stream->cam_name);
		} else if (stream->stream_type & streamType::RGB) {
			// ONLY RGB
			env->vis.scn.flags[mjRND_SEGMENT] = 0;
			rendered = renderAndPubEnv(env, true, false, stream->rgb_pub, stream->depth_pub, stream->width, stream->height,
			                           stream->cam_name);
		} else {
			// Only DEPTH or only SEGMENTED
			env->vis.scn.flags[mjRND_IDCOLOR] = stream->use_segid;
			env->vis.scn.flags[mjRND_SEGMENT] = 1;
			rendered                          = renderAndPubEnv(env, (stream->stream_type & streamType::SEGMENTED),
                                    (stream->stream_type & streamType::DEPTH), stream->segment_pub, stream->depth_pub,
                                    stream->width, stream->height, stream->cam_name);
		}

		if (rendered) {
			stream->publishCameraInfo();
		}
	}

	env->vis.cam.type                 = type_backup;
	env->vis.cam.fixedcamid           = cam_id_backup;
	env->vis.scn.flags[mjRND_SEGMENT] = segment_backup;
	env->vis.scn.flags[mjRND_IDCOLOR] = segid_backup;
}

void onModelLoad(MujocoEnvPtr env, bool align)
{
	if (settings_.headless) {
		return;
	}

	ROS_DEBUG_NAMED("mujoco_ui", "Clearing perturb...");
	pert_.active     = 0;
	pert_.select     = 0;
	pert_.skinselect = -1;

	if (align) {
		alignScale(env);
	}

	std::lock_guard<std::mutex> lk(render_mtx);
	glfwMakeContextCurrent(main_window_);
	mjr_makeContext(env->model.get(), &free_context_, 50 * (settings_.font + 1));

	mjv_updateScene(env->model.get(), env->data.get(), &vopt_, &pert_, &free_camera_, mjCAT_ALL, &free_scene_);

	char title[200] = "Simulate : ";
	mju::strcat_arr(title, env->model->names);
	glfwSetWindowTitle(main_window_, title);

	// Set keyframe range and divisions
	ui0_.sect[SECT_SIMULATION].item[5].slider.range[0]  = 0;
	ui0_.sect[SECT_SIMULATION].item[5].slider.range[1]  = mjMAX(0, env->model->nkey - 1);
	ui0_.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, env->model->nkey - 1);

	// Rebuild UI Sections
	makeSections(env);

	// Full UI update
	uiModify(main_window_, &ui0_, &uistate_, &free_context_);
	uiModify(main_window_, &ui1_, &uistate_, &free_context_);

	ROS_DEBUG_NAMED("mujoco ui", "updating settings ...");
	// model pointers of all envs point to the same model instance
	updateSettings(env);
	ROS_DEBUG_NAMED("mujoco ui", "settings updated ...");
}

void profilerInit(void)
{
	int i, n;

	// set figures to default
	mjv_defaultFigure(&figconstraint_);
	mjv_defaultFigure(&figcost_);
	mjv_defaultFigure(&figtimer_);
	mjv_defaultFigure(&figsize_);

	// titles
	mju::strcpy_arr(figconstraint_.title, "Counts");
	mju::strcpy_arr(figcost_.title, "Convergence (log 10)");
	mju::strcpy_arr(figsize_.title, "Dimensions");
	mju::strcpy_arr(figtimer_.title, "CPU time (msec)");

	// x-labels
	mju::strcpy_arr(figconstraint_.xlabel, "Solver iteration");
	mju::strcpy_arr(figcost_.xlabel, "Solver iteration");
	mju::strcpy_arr(figsize_.xlabel, "Video Frame");
	mju::strcpy_arr(figtimer_.xlabel, "Video Frame");

	// y-tick number formats
	mju::strcpy_arr(figconstraint_.yformat, "%.0f");
	mju::strcpy_arr(figcost_.yformat, "%.1f");
	mju::strcpy_arr(figsize_.yformat, "%.0f");
	mju::strcpy_arr(figtimer_.yformat, "%.2f");

	// colors
	figconstraint_.figurergba[0] = 0.1f;
	figcost_.figurergba[2]       = 0.2f;
	figsize_.figurergba[0]       = 0.1f;
	figtimer_.figurergba[2]      = 0.2f;
	figconstraint_.figurergba[3] = 0.5f;
	figcost_.figurergba[3]       = 0.5f;
	figsize_.figurergba[3]       = 0.5f;
	figtimer_.figurergba[3]      = 0.5f;

	// legends
	mju::strcpy_arr(figconstraint_.linename[0], "total");
	mju::strcpy_arr(figconstraint_.linename[1], "active");
	mju::strcpy_arr(figconstraint_.linename[2], "changed");
	mju::strcpy_arr(figconstraint_.linename[3], "evals");
	mju::strcpy_arr(figconstraint_.linename[4], "updates");
	mju::strcpy_arr(figcost_.linename[0], "improvement");
	mju::strcpy_arr(figcost_.linename[1], "gradient");
	mju::strcpy_arr(figcost_.linename[2], "lineslope");
	mju::strcpy_arr(figsize_.linename[0], "dof");
	mju::strcpy_arr(figsize_.linename[1], "body");
	mju::strcpy_arr(figsize_.linename[2], "constraint");
	mju::strcpy_arr(figsize_.linename[3], "sqrt(nnz)");
	mju::strcpy_arr(figsize_.linename[4], "contact");
	mju::strcpy_arr(figsize_.linename[5], "iteration");
	mju::strcpy_arr(figtimer_.linename[0], "total");
	mju::strcpy_arr(figtimer_.linename[1], "collision");
	mju::strcpy_arr(figtimer_.linename[2], "prepare");
	mju::strcpy_arr(figtimer_.linename[3], "solve");
	mju::strcpy_arr(figtimer_.linename[4], "other");

	// grid sizes
	figconstraint_.gridsize[0] = 5;
	figconstraint_.gridsize[1] = 5;
	figcost_.gridsize[0]       = 5;
	figcost_.gridsize[1]       = 5;
	figsize_.gridsize[0]       = 3;
	figsize_.gridsize[1]       = 5;
	figtimer_.gridsize[0]      = 3;
	figtimer_.gridsize[1]      = 5;

	// minimum ranges
	figconstraint_.range[0][0] = 0;
	figconstraint_.range[0][1] = 20;
	figconstraint_.range[1][0] = 0;
	figconstraint_.range[1][1] = 80;
	figcost_.range[0][0]       = 0;
	figcost_.range[0][1]       = 20;
	figcost_.range[1][0]       = -15;
	figcost_.range[1][1]       = 5;
	figsize_.range[0][0]       = -200;
	figsize_.range[0][1]       = 0;
	figsize_.range[1][0]       = 0;
	figsize_.range[1][1]       = 100;
	figtimer_.range[0][0]      = -200;
	figtimer_.range[0][1]      = 0;
	figtimer_.range[1][0]      = 0;
	figtimer_.range[1][1]      = 0.4f;

	// init x axis on history figures (do not show yet)
	for (n = 0; n < 6; n++)
		for (i = 0; i < mjMAXLINEPNT; i++) {
			figtimer_.linedata[n][2 * i] = (float)-i;
			figsize_.linedata[n][2 * i]  = (float)-i;
		}
}

void profilerUpdate(MujocoEnvPtr env)
{
	int i, n;

	// update constraint figure
	figconstraint_.linepnt[0] = mjMIN(mjMIN(env->data->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 5; i++) {
		figconstraint_.linepnt[i] = figconstraint_.linepnt[0];
	}
	if (env->model->opt.solver == mjSOL_PGS) {
		figconstraint_.linepnt[3] = 0;
		figconstraint_.linepnt[4] = 0;
	}
	if (env->model->opt.solver == mjSOL_CG) {
		figconstraint_.linepnt[4] = 0;
	}
	for (i = 0; i < figconstraint_.linepnt[0]; i++) {
		// x
		figconstraint_.linedata[0][2 * i] = (float)i;
		figconstraint_.linedata[1][2 * i] = (float)i;
		figconstraint_.linedata[2][2 * i] = (float)i;
		figconstraint_.linedata[3][2 * i] = (float)i;
		figconstraint_.linedata[4][2 * i] = (float)i;

		// y
		figconstraint_.linedata[0][2 * i + 1] = (float)env->data->nefc;
		figconstraint_.linedata[1][2 * i + 1] = (float)env->data->solver[i].nactive;
		figconstraint_.linedata[2][2 * i + 1] = (float)env->data->solver[i].nchange;
		figconstraint_.linedata[3][2 * i + 1] = (float)env->data->solver[i].neval;
		figconstraint_.linedata[4][2 * i + 1] = (float)env->data->solver[i].nupdate;
	}

	// update cost figure
	figcost_.linepnt[0] = mjMIN(mjMIN(env->data->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 3; i++) {
		figcost_.linepnt[i] = figcost_.linepnt[0];
	}
	if (env->model->opt.solver == mjSOL_PGS) {
		figcost_.linepnt[1] = 0;
		figcost_.linepnt[2] = 0;
	}

	for (i = 0; i < figcost_.linepnt[0]; i++) {
		// x
		figcost_.linedata[0][2 * i] = (float)i;
		figcost_.linedata[1][2 * i] = (float)i;
		figcost_.linedata[2][2 * i] = (float)i;

		// y
		figcost_.linedata[0][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, env->data->solver[i].improvement));
		figcost_.linedata[1][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, env->data->solver[i].gradient));
		figcost_.linedata[2][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, env->data->solver[i].lineslope));
	}

	// get timers: total, collision, prepare, solve, other
	mjtNum total = env->data->timer[mjTIMER_STEP].duration;
	int number   = env->data->timer[mjTIMER_STEP].number;
	if (!number) {
		total  = env->data->timer[mjTIMER_FORWARD].duration;
		number = env->data->timer[mjTIMER_FORWARD].number;
	}
	number         = mjMAX(1, number);
	float tdata[5] = { (float)(total / number), (float)(env->data->timer[mjTIMER_POS_COLLISION].duration / number),
		                (float)(env->data->timer[mjTIMER_POS_MAKE].duration / number) +
		                    (float)(env->data->timer[mjTIMER_POS_PROJECT].duration / number),
		                (float)(env->data->timer[mjTIMER_CONSTRAINT].duration / number), 0 };
	tdata[4]       = tdata[0] - tdata[1] - tdata[2] - tdata[3];

	// update figtimer_
	int pnt = mjMIN(201, figtimer_.linepnt[0] + 1);
	for (n = 0; n < 5; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figtimer_.linedata[n][2 * i + 1] = figtimer_.linedata[n][2 * i - 1];
		}

		// assign new
		figtimer_.linepnt[n]     = pnt;
		figtimer_.linedata[n][1] = tdata[n];
	}

	// get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
	float sdata[6] = { (float)env->model->nv,  (float)env->model->nbody,
		                (float)env->data->nefc, (float)mju_sqrt((mjtNum)env->data->solver_nnz),
		                (float)env->data->ncon, (float)env->data->solver_iter };

	// update figsize_
	pnt = mjMIN(201, figsize_.linepnt[0] + 1);
	for (n = 0; n < 6; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figsize_.linedata[n][2 * i + 1] = figsize_.linedata[n][2 * i - 1];
		}

		// assign new
		figsize_.linepnt[n]     = pnt;
		figsize_.linedata[n][1] = sdata[n];
	}
}

void profilerShow(MujocoEnvPtr env, mjrRect rect)
{
	mjrRect viewport = { rect.left + rect.width - rect.width / 4, rect.bottom, rect.width / 4, rect.height / 4 };
	mjr_figure(viewport, &figtimer_, &free_context_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figsize_, &free_context_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figcost_, &free_context_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figconstraint_, &free_context_);
}

void sensorInit(void)
{
	// set figure to default
	mjv_defaultFigure(&figsensor_);
	figsensor_.figurergba[3] = 0.5f;

	// set flags
	figsensor_.flg_extend    = 1;
	figsensor_.flg_barplot   = 1;
	figsensor_.flg_symmetric = 1;

	// title
	mju::strcpy_arr(figsensor_.title, "Sensor data");

	// y-tick number format
	mju::strcpy_arr(figsensor_.yformat, "%.1f");

	// grid size
	figsensor_.gridsize[0] = 2;
	figsensor_.gridsize[1] = 3;

	// minimum range
	figsensor_.range[0][0] = 0;
	figsensor_.range[0][0] = 0;
	figsensor_.range[1][0] = -1;
	figsensor_.range[1][1] = 1;
}

// Update sensor figure
void sensorUpdate(MujocoEnvPtr env)
{
	static const int maxline = 10;

	// clear linepnt
	for (int i = 0; i < maxline; i++) {
		figsensor_.linepnt[i] = 0;
	}

	// start with line 0
	int lineid = 0;

	// loop over sensors
	for (int n = 0; n < env->model->nsensor; n++) {
		// go to next line if type is different
		if (n > 0 && env->model->sensor_type[n] != env->model->sensor_type[n - 1])
			lineid = mjMIN(lineid + 1, maxline - 1);

		// get info about this sensor
		mjtNum cutoff = (env->model->sensor_cutoff[n] > 0 ? env->model->sensor_cutoff[n] : 1);
		int adr       = env->model->sensor_adr[n];
		int dim       = env->model->sensor_dim[n];

		// data pointer in line
		int p = figsensor_.linepnt[lineid];

		// fill in data for this sensor
		for (int i = 0; i < dim; i++) {
			// check size
			if ((p + 2 * i) > mjMAXLINEPNT / 2)
				break;

			// x
			figsensor_.linedata[lineid][2 * p + 4 * i]     = (float)(adr + i);
			figsensor_.linedata[lineid][2 * p + 4 * i + 2] = (float)(adr + i);

			// y
			figsensor_.linedata[lineid][2 * p + 4 * i + 1] = 0;
			figsensor_.linedata[lineid][2 * p + 4 * i + 3] = (float)(env->data->sensordata[adr + i] / cutoff);
		}

		// update linepnt
		figsensor_.linepnt[lineid] = mjMIN(mjMAXLINEPNT - 1, figsensor_.linepnt[lineid] + 2 * dim);
	}
}

// Show sensor figure
void sensorShow(MujocoEnvPtr env, mjrRect rect)
{
	// constant width with and without profiler
	int width = settings_.profiler ? rect.width / 3 : rect.width / 4;

	// render figure on the right
	mjrRect viewport = { rect.left + rect.width - width, rect.bottom, width, rect.height / 3 };
	mjr_figure(viewport, &figsensor_, &free_context_);
}

// Prepare info text
void infotext(MujocoEnvPtr env, char (&title)[kBufSize], char (&content)[kBufSize], double interval)
{
	char tmp[20];

	// Compute solver error
	mjtNum solerr = 0;
	if (env->data->solver_iter) {
		int ind = mjMIN(env->data->solver_iter - 1, mjNSOLVER - 1);
		solerr  = mju_min(env->data->solver[ind].improvement, env->data->solver[ind].gradient);
		if (solerr == 0) {
			solerr = mju_max(env->data->solver[ind].improvement, env->data->solver[ind].gradient);
		}
	}
	solerr = mju_log10(mju_max(mjMINVAL, solerr));

	mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver\nFPS\nMemory");
	mju::sprintf_arr(content, "%-9.3f\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.2g of %s", env->data->time,
	                 env->data->nefc, env->data->ncon,
	                 settings_.run.load() ?
	                     env->data->timer[mjTIMER_STEP].duration / mjMAX(1, env->data->timer[mjTIMER_STEP].number) :
	                     env->data->timer[mjTIMER_FORWARD].duration / mjMAX(1, env->data->timer[mjTIMER_FORWARD].number),
	                 solerr, env->data->solver_iter, 1 / interval,
	                 env->data->maxuse_arena / (double)(env->data->nstack * sizeof(mjtNum)),
	                 env->data->maxuse_con / (double)env->model->nconmax,
	                 mju_writeNumBytes(env->data->nstack * sizeof(mjtNum)));

	// Add energy if enabled
	if (mjENABLED_ros(env->model, mjENBL_ENERGY)) {
		mju::sprintf_arr(tmp, "\n%.3f", env->data->energy[0] + env->data->energy[1]);
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nEnergy");
	}

	// Add FwdInv if enabled
	if (mjENABLED_ros(env->model, mjENBL_FWDINV)) {
		mju::sprintf_arr(tmp, "\n%.1f %.1f", mju_log10(mju_max(mjMINVAL, env->data->solver_fwdinv[0])),
		                 mju_log10(mju_max(mjMINVAL, env->data->solver_fwdinv[1])));
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nFwdInv");
	}
}

// Sprintf forwarding, to avoid compiler warning in x-macro
void printField(char (&str)[mjMAXUINAME], void *ptr)
{
	mju::sprintf_arr(str, "%g", *(mjtNum *)ptr);
}

// Update watch
void watch(MujocoEnvPtr env)
{
	// clear
	ui0_.sect[SECT_WATCH].item[2].multi.nelem = 1;
	mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

	// prepare symbols needed by xmacro
	MJDATA_POINTERS_PREAMBLE(env->model);

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                                                        \
	if (!mju::strcmp_arr(#NAME, settings_.field) && !mju::strcmp_arr(#TYPE, "mjtNum")) {              \
		if (settings_.index >= 0 && settings_.index < env->model->NR * NC) {                           \
			printField(ui0_.sect[SECT_WATCH].item[2].multi.name[0], env->data->NAME + settings_.index); \
		} else {                                                                                       \
			mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");              \
		}                                                                                              \
		return;                                                                                        \
	}

	MJDATA_POINTERS
#undef X
}

// Physics section of UI
void makePhysics(MujocoEnvPtr env, int oldstate)
{
	int i;

	mjuiDef defPhysics[]     = { { mjITEM_SECTION, "Physics", oldstate, nullptr, "AP" },
                            { mjITEM_SELECT, "Integrator", 2, &(env->model->opt.integrator),
                              "Euler\nRK4\nimplicit\nimplicitfast" },
                            { mjITEM_SELECT, "Collision", 2, &(env->model->opt.collision), "All\nPair\nDynamic" },
                            { mjITEM_SELECT, "Cone", 2, &(env->model->opt.cone), "Pyramidal\nElliptic" },
                            { mjITEM_SELECT, "Jacobian", 2, &(env->model->opt.jacobian), "Dense\nSparse\nAuto" },
                            { mjITEM_SELECT, "Solver", 2, &(env->model->opt.solver), "PGS\nCG\nNewton" },
                            { mjITEM_SEPARATOR, "Algorithmic Parameters", 1 },
                            { mjITEM_EDITNUM, "Timestep", 2, &(env->model->opt.timestep), "1 0 1" },
                            { mjITEM_EDITINT, "Iterations", 2, &(env->model->opt.iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Tolerance", 2, &(env->model->opt.tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "Noslip Iter", 2, &(env->model->opt.noslip_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Noslip Tol", 2, &(env->model->opt.noslip_tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "MPR Iter", 2, &(env->model->opt.mpr_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "MPR Tol", 2, &(env->model->opt.mpr_tolerance), "1 0 1" },
                            { mjITEM_EDITNUM, "API Rate", 2, &(env->model->opt.apirate), "1 0 1000" },
                            { mjITEM_SEPARATOR, "Physical Parameters", 1 },
                            { mjITEM_EDITNUM, "Gravity", 2, env->model->opt.gravity, "3" },
                            { mjITEM_EDITNUM, "Wind", 2, env->model->opt.wind, "3" },
                            { mjITEM_EDITNUM, "Magnetic", 2, env->model->opt.magnetic, "3" },
                            { mjITEM_EDITNUM, "Density", 2, &(env->model->opt.density), "1" },
                            { mjITEM_EDITNUM, "Viscosity", 2, &(env->model->opt.viscosity), "1" },
                            { mjITEM_EDITNUM, "Imp Ratio", 2, &(env->model->opt.impratio), "1" },
                            { mjITEM_SEPARATOR, "Disable Flags", 1 },
                            { mjITEM_END } };
	mjuiDef defEnableFlags[] = { { mjITEM_SEPARATOR, "Enable Flags", 1 }, { mjITEM_END } };
	mjuiDef defOverride[]    = { { mjITEM_SEPARATOR, "Contact Override", 1 },
                             { mjITEM_EDITNUM, "Margin", 2, &(env->model->opt.o_margin), "1" },
                             { mjITEM_EDITNUM, "Sol Imp", 2, &(env->model->opt.o_solimp), "5" },
                             { mjITEM_EDITNUM, "Sol Ref", 2, &(env->model->opt.o_solref), "2" },
                             { mjITEM_END } };

	// add physics
	mjui_add(&ui0_, defPhysics);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKINT, "", 2, nullptr, "" }, { mjITEM_END } };
	for (i = 0; i < mjNDISABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
		defFlag[0].pdata = settings_.disable + i;
		mjui_add(&ui0_, defFlag);
	}
	mjui_add(&ui0_, defEnableFlags);
	for (i = 0; i < mjNENABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
		defFlag[0].pdata = settings_.enable + i;
		mjui_add(&ui0_, defFlag);
	}

	// add contact override
	mjui_add(&ui0_, defOverride);
}

// Make rendering section of UI
void makeRendering(MujocoEnvPtr env, int oldstate)
{
	int i;
	unsigned long int j;

	mjuiDef defRendering[] = { { mjITEM_SECTION, "Rendering", oldstate, nullptr, "AR" },
		                        { mjITEM_SELECT, "Camera", 2, &(settings_.camera), "Free\nTracking" },
		                        { mjITEM_SELECT, "Label", 2, &(vopt_.label),
		                          "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSele"
		                          "ction\nSel Pnt\nContact\nForce" },
		                        { mjITEM_SELECT, "Frame", 2, &(vopt_.frame),
		                          "None\nBody\nGeom\nSite\nCamera\nLight\nWorld" },
		                        { mjITEM_BUTTON, "Copy camera", 2, nullptr, "" },
		                        {
		                            mjITEM_SEPARATOR,
		                            "Model Elements",
		                            1,
		                        },
		                        { mjITEM_END } };
	mjuiDef defOpenGL[]    = { { mjITEM_SEPARATOR, "OpenGL Effects", 1 }, { mjITEM_END } };

	// add model cameras, up to UI limit
	for (i = 0; i < mjMIN(env->model->ncam, mjMAXUIMULTI - 2); i++) {
		// prepare name
		char camname[mjMAXUITEXT] = "\n";
		if (env->model->names[env->model->name_camadr[i]]) {
			mju::strcat_arr(camname, env->model->names + env->model->name_camadr[i]);
		} else {
			mju::sprintf_arr(camname, "\nCamera %d", i);
		}

		// check string length
		if (mju::strlen_arr(camname) + mju::strlen_arr(defRendering[1].other) >= mjMAXUITEXT - 1) {
			break;
		}

		// add camera
		mju::strcat_arr(defRendering[1].other, camname);
	}

	// add rendering standard
	mjui_add(&ui0_, defRendering);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKBYTE, "", 2, nullptr, "" }, { mjITEM_END } };
	for (i = 0; i < mjNVISFLAG; i++) {
		// set name, remove "&"
		mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);
		for (j = 0; j < strlen(mjVISSTRING[i][0]); j++) {
			if (mjVISSTRING[i][0][j] == '&') {
				mju_strncpy(defFlag[0].name + j, mjVISSTRING[i][0] + j + 1, mju::sizeof_arr(defFlag[0].name) - j);
				break;
			}
		}

		// set shortcut and data
		mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
		defFlag[0].pdata = vopt_.flags + i;
		mjui_add(&ui0_, defFlag);
	}

	env->model->vis.global.treedepth = 0;
	mjuiDef defTree[] = { { mjITEM_SLIDERINT, "Tree depth", 2, &env->model->vis.global.treedepth, "-1 15" },
		                   { mjITEM_END } };
	mjui_add(&ui0_, defTree);

	mjui_add(&ui0_, defOpenGL);
	for (i = 0; i < mjNRNDFLAG; i++) {
		mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);
		if (mjRNDSTRING[i][2][0])
			mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
		defFlag[0].pdata = free_scene_.flags + i;
		mjui_add(&ui0_, defFlag);
	}
}

// Make group section UI
void makeGroup(MujocoEnvPtr env, int oldstate)
{
	mjuiDef defGroup[] = { { mjITEM_SECTION, "Group enable", oldstate, nullptr, "AG" },
		                    { mjITEM_SEPARATOR, "Geom groups", 1 },
		                    { mjITEM_CHECKBYTE, "Geom 0", 2, vopt_.geomgroup, " 0" },
		                    { mjITEM_CHECKBYTE, "Geom 1", 2, vopt_.geomgroup + 1, " 1" },
		                    { mjITEM_CHECKBYTE, "Geom 2", 2, vopt_.geomgroup + 2, " 2" },
		                    { mjITEM_CHECKBYTE, "Geom 3", 2, vopt_.geomgroup + 3, " 3" },
		                    { mjITEM_CHECKBYTE, "Geom 4", 2, vopt_.geomgroup + 4, " 4" },
		                    { mjITEM_CHECKBYTE, "Geom 5", 2, vopt_.geomgroup + 5, " 5" },
		                    { mjITEM_SEPARATOR, "Site groups", 1 },
		                    { mjITEM_CHECKBYTE, "Site 0", 2, vopt_.sitegroup, "S0" },
		                    { mjITEM_CHECKBYTE, "Site 1", 2, vopt_.sitegroup + 1, "S1" },
		                    { mjITEM_CHECKBYTE, "Site 2", 2, vopt_.sitegroup + 2, "S2" },
		                    { mjITEM_CHECKBYTE, "Site 3", 2, vopt_.sitegroup + 3, "S3" },
		                    { mjITEM_CHECKBYTE, "Site 4", 2, vopt_.sitegroup + 4, "S4" },
		                    { mjITEM_CHECKBYTE, "Site 5", 2, vopt_.sitegroup + 5, "S5" },
		                    { mjITEM_SEPARATOR, "Joint groups", 1 },
		                    { mjITEM_CHECKBYTE, "Joint 0", 2, vopt_.jointgroup, "" },
		                    { mjITEM_CHECKBYTE, "Joint 1", 2, vopt_.jointgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Joint 2", 2, vopt_.jointgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Joint 3", 2, vopt_.jointgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Joint 4", 2, vopt_.jointgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Joint 5", 2, vopt_.jointgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Tendon groups", 1 },
		                    { mjITEM_CHECKBYTE, "Tendon 0", 2, vopt_.tendongroup, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 1", 2, vopt_.tendongroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 2", 2, vopt_.tendongroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 3", 2, vopt_.tendongroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 4", 2, vopt_.tendongroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 5", 2, vopt_.tendongroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Actuator groups", 1 },
		                    { mjITEM_CHECKBYTE, "Actuator 0", 2, vopt_.actuatorgroup, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 1", 2, vopt_.actuatorgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 2", 2, vopt_.actuatorgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 3", 2, vopt_.actuatorgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 4", 2, vopt_.actuatorgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 5", 2, vopt_.actuatorgroup + 5, "" },
		                    { mjITEM_END } };

	mjui_add(&ui0_, defGroup);
}

// Make joint section of UI
void makeJoint(MujocoEnvPtr env, int oldstate)
{
	int i;

	mjuiDef defJoint[] = {
		{ mjITEM_SECTION, "Joint", oldstate, nullptr, "AJ" },
		{ mjITEM_END },
	};
	mjuiDef defSlider[] = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// add section
	mjui_add(&ui1_, defJoint);
	defSlider[0].state = 4;

	// add scalar joints, exit if UI limit reached
	int itemcnt = 0;
	for (i = 0; i < env->model->njnt && itemcnt < mjMAXUIITEM; i++) {
		if ((env->model->jnt_type[i] == mjJNT_HINGE || env->model->jnt_type[i] == mjJNT_SLIDE)) {
			// skip if joint group is disabled
			if (!vopt_.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, env->model->jnt_group[i]))]) {
				continue;
			}

			// set data and name
			defSlider[0].pdata = env->data->qpos + env->model->jnt_qposadr[i];
			if (env->model->names[env->model->name_jntadr[i]]) {
				mju::strcpy_arr(defSlider[0].name, env->model->names + env->model->name_jntadr[i]);
			} else {
				mju::sprintf_arr(defSlider[0].name, "joint %d", i);
			}

			// set range
			if (env->model->jnt_limited[i]) {
				mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", env->model->jnt_range[2 * i],
				                 env->model->jnt_range[2 * i + 1]);
			} else if (env->model->jnt_type[i] == mjJNT_SLIDE) {
				mju::strcpy_arr(defSlider[0].other, "-1 1");
			} else {
				mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
			}

			// add and count
			mjui_add(&ui1_, defSlider);
			itemcnt++;
		}
	}
}

// Make control section of UI
void makeControl(MujocoEnvPtr env, int oldstate)
{
	int i;

	mjuiDef defControl[] = { { mjITEM_SECTION, "Control", oldstate, nullptr, "AC" },
		                      { mjITEM_BUTTON, "Clear all", 2 },
		                      { mjITEM_END } };
	mjuiDef defSlider[]  = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// Add section
	mjui_add(&ui1_, defControl);
	defSlider[0].state = 2;

	// Add controls, exit if UI limit reached (Clear button already added)
	int itemcnt = 1;
	for (i = 0; i < env->model->nu && itemcnt < mjMAXUIITEM; i++) {
		// Skip if actuator group is disabled
		if (!vopt_.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, env->model->actuator_group[i]))]) {
			continue;
		}

		// set data and name
		defSlider[0].pdata = env->data->ctrl + i;
		if (env->model->names[env->model->name_actuatoradr[i]]) {
			mju::strcpy_arr(defSlider[0].name, env->model->names + env->model->name_actuatoradr[i]);
		} else {
			mju::sprintf_arr(defSlider[0].name, "control %d", i);
		}

		// set range
		if (env->model->actuator_ctrllimited[i]) {
			mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", env->model->actuator_ctrlrange[2 * i + 1]);
		} else {
			mju::strcpy_arr(defSlider[0].other, "-1 1");
		}

		// add and count
		mjui_add(&ui1_, defSlider);
		itemcnt++;
	}
}

// Make model-dependent UI sections
void makeSections(MujocoEnvPtr env)
{
	int i;

	// get section open-close state, UI 0
	int oldstate0[NSECT0];
	for (i = 0; i < NSECT0; i++) {
		oldstate0[i] = 0;
		if (ui0_.nsect > i) {
			oldstate0[i] = ui0_.sect[i].state;
		}
	}

	// get section open-close state, UI 1
	int oldstate1[NSECT1];
	for (i = 0; i < NSECT1; i++) {
		oldstate1[i] = 0;
		if (ui1_.nsect > i) {
			oldstate1[i] = ui1_.sect[i].state;
		}
	}

	// clear model-dependent sections of UI
	ui0_.nsect = SECT_PHYSICS;
	ui1_.nsect = 0;

	// make
	makePhysics(env, oldstate0[SECT_PHYSICS]);
	makeRendering(env, oldstate0[SECT_RENDERING]);
	makeGroup(env, oldstate0[SECT_GROUP]);
	makeJoint(env, oldstate1[SECT_JOINT]);
	makeControl(env, oldstate1[SECT_CONTROL]);
}

// Align and scale view
void alignScale(MujocoEnvPtr env)
{
	// autoscale
	free_camera_.lookat[0] = env->model->stat.center[0];
	free_camera_.lookat[1] = env->model->stat.center[1];
	free_camera_.lookat[2] = env->model->stat.center[2];
	free_camera_.distance  = 1.5 * env->model->stat.extent;

	// set to free camera
	free_camera_.type = mjCAMERA_FREE;
}

// copy qpos to clipboard as key
void copyKey(MujocoEnvPtr env)
{
	char clipboard[5000] = "<key qpos='";
	char buf[200];

	// prepare string
	for (int i = 0; i < env->model->nq; i++) {
		mju::sprintf_arr(buf, i == env->model->nq - 1 ? "%g" : "%g ", env->data->qpos[i]);
		mju::strcat_arr(clipboard, buf);
	}
	mju::strcat_arr(clipboard, "'/>");

	// copy to clipboard
	glfwSetClipboardString(main_window_, clipboard);
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
	return (mjtNum)(1000 * glfwGetTime());
}

// Clear all timers
void clearTimers(mjDataPtr data)
{
	for (int i = 0; i < mjNTIMER; i++) {
		data->timer[i].duration = 0;
		data->timer[i].number   = 0;
	}
}

// print current camera as MJCF specification
void printCamera(mjvGLCamera *camera)
{
	mjtNum camright[3];
	mjtNum camforward[3];
	mjtNum camup[3];
	mju_f2n(camforward, camera[0].forward, 3);
	mju_f2n(camup, camera[0].up, 3);
	mju_cross(camright, camforward, camup);
	std::printf("<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
	            (camera[0].pos[0] + camera[1].pos[0]) / 2, (camera[0].pos[1] + camera[1].pos[1]) / 2,
	            (camera[0].pos[2] + camera[1].pos[2]) / 2, camright[0], camright[1], camright[2], camera[0].up[0],
	            camera[0].up[1], camera[0].up[2]);
}

// Update UI 0 when MuJoCo structures change (except for joint sliders)
void updateSettings(MujocoEnvPtr env)
{
	int i;

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating physics");
	// physics flags
	for (i = 0; i < mjNDISABLE; i++) {
		settings_.disable[i] = ((env->model->opt.disableflags & (i << i)) != 0);
	}
	for (i = 0; i < mjNENABLE; i++) {
		settings_.enable[i] = ((env->model->opt.enableflags & (1 << 1)) != 0);
	}

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating cam");
	// camera
	if (env->vis.cam.type == mjCAMERA_FIXED) {
		settings_.camera = 2 + env->vis.cam.fixedcamid;
	} else if (env->vis.cam.type == mjCAMERA_TRACKING) {
		settings_.camera = 1;
	} else {
		settings_.camera = 0;
	}

	// update UI
	if (settings_.headless)
		mjui_update(-1, -1, &ui0_, &uistate_, &free_context_);
}

int uiPredicate(int category, void *userdata)
{
	MujocoEnvPtr env = *(MujocoEnvPtr *)userdata;
	switch (category) {
		case 2: // require model
			return (env->model != nullptr);

		case 3: //
			return (env->model && env->model->nkey);

		case 4:
			return (env->model && !settings_.run.load());

		default:
			return 1;
	}
}

// Set window layout
void uiLayout(mjuiState *state)
{
	mjrRect *rect    = state->rect;
	MujocoEnvPtr env = *(MujocoEnvPtr *)state->userdata;

	// set number of rectangles
	state->nrect = 4;

	// rect 0: entire framebuffer
	rect[0].left   = 0;
	rect[0].bottom = 0;
	glfwGetFramebufferSize(main_window_, &rect[0].width, &rect[0].height);

	// rect 1: UI 0
	rect[1].left   = 0;
	rect[1].width  = settings_.ui0 ? ui0_.width : 0;
	rect[1].bottom = 0;
	rect[1].height = rect[0].height;

	// rect 2: UI 1
	rect[2].width  = settings_.ui1 ? ui1_.width : 0;
	rect[2].left   = mjMAX(0, rect[0].width - rect[2].width);
	rect[2].bottom = 0;
	rect[2].height = rect[0].height;

	// rect 3: 3D plot (everything else is an overlay)
	rect[3].left   = rect[1].width;
	rect[3].width  = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
	rect[3].bottom = 0;
	rect[3].height = rect[0].height;
}

// Handle UI event
void uiEvent(mjuiState *state)
{
	std::lock_guard<std::mutex> lk(render_mtx);
	glfwMakeContextCurrent(main_window_);

	int i;
	char err[200];
	MujocoEnvPtr env = *(MujocoEnvPtr *)state->userdata;

	// call UI 0 if event is directed to it
	if ((state->dragrect == ui0_.rectid) || (state->dragrect == 0 && state->mouserect == ui0_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// process UI event
		mjuiItem *it = mjui_event(&ui0_, state, &free_context_);

		// file section
		if (it && it->sectionid == SECT_FILE) {
			switch (it->itemid) {
				case 0: // save xml
					if (!mj_saveLastXML("mjmodel.xml", env->model.get(), err, 200))
						ROS_ERROR("Save XML error: %s", err);
					break;

				case 1: // Save mjb
					mj_saveModel(env->model.get(), "mjmodel.mjb", nullptr, 0);
					break;

				case 2: // print model
					mj_printModel(env->model.get(), "MJMODEL.TXT");
					break;

				case 3: // print data
					mj_printData(env->model.get(), env->data.get(), "MJDATA.TXT");
					break;

				case 4: // Quit
					settings_.exitrequest.store(1);
					break;
			}
		}

		// option section
		else if (it && it->sectionid == SECT_OPTION) {
			switch (it->itemid) {
				case 0: // Spacing
					ui0_.spacing = mjui_themeSpacing(settings_.spacing);
					ui1_.spacing = mjui_themeSpacing(settings_.spacing);
					break;

				case 1: // Color
					ui0_.color = mjui_themeColor(settings_.color);
					ui1_.color = mjui_themeColor(settings_.color);
					break;

				case 2: // Font
					mjr_changeFont(50 * (settings_.font + 1), &free_context_);
					break;

				case 9: // Full screen
					if (glfwGetWindowMonitor(main_window_)) {
						// restore window from saved data
						glfwSetWindowMonitor(main_window_, nullptr, windowpos_[0], windowpos_[1], windowsize_[0],
						                     windowsize_[1], 0);
					} else { // currently windowed: switch to fullscreen
						// save window data
						glfwGetWindowPos(main_window_, windowpos_, windowpos_ + 1);
						glfwGetWindowSize(main_window_, windowsize_, windowsize_ + 1);

						// switch
						glfwSetWindowMonitor(main_window_, glfwGetPrimaryMonitor(), 0, 0, vmode_.width, vmode_.height,
						                     vmode_.refreshRate);
					}

					// reinstate vsync, just in case
					glfwSwapInterval(settings_.vsync);
					break;

				case 10: // Vertical sync
					glfwSwapInterval(settings_.vsync);
					break;
			}

			// modify UI
			uiModify(main_window_, &ui0_, state, &free_context_);
			uiModify(main_window_, &ui1_, state, &free_context_);
		}

		// simulation section
		else if (it && it->sectionid == SECT_SIMULATION) {
			switch (it->itemid) {
				case 1: // reset
					settings_.resetrequest.store(1);
					break;

				case 2: // Reload
					settings_.loadrequest.store(3);
					break;

				case 3: // Align
					alignScale(env);
					updateSettings(env);
					break;

				case 4: // Copy Pose
					copyKey(env);
					break;

				case 5: // Adjust key
				case 6: // Reset to key
					// REVIEW: fully disable resetting to keypoints?
					i = settings_.key;
					mju_copy(env->data->qpos, env->model->key_qpos + i * env->model->nq, env->model->nq);
					mju_copy(env->data->qvel, env->model->key_qvel + i * env->model->nv, env->model->nv);
					mju_copy(env->data->act, env->model->key_act + i * env->model->na, env->model->na);
					mju_copy(env->data->mocap_pos, env->model->key_mpos + i * 3 * env->model->nmocap,
					         3 * env->model->nmocap);
					mju_copy(env->data->mocap_quat, env->model->key_mquat + i * 4 * env->model->nmocap,
					         4 * env->model->nmocap);
					mju_copy(env->data->ctrl, env->model->key_ctrl + i * env->model->nu, env->model->nu);
					mj_forward(env->model.get(), env->data.get());
					profilerUpdate(env);
					sensorUpdate(env);
					updateSettings(env);
					break;

				case 7: // Set key
					i                       = settings_.key;
					env->model->key_time[i] = env->data->time;
					mju_copy(env->model->key_qpos + i * env->model->nq, env->data->qpos, env->model->nq);
					mju_copy(env->model->key_qvel + i * env->model->nv, env->data->qvel, env->model->nv);
					mju_copy(env->model->key_act + i * env->model->na, env->data->act, env->model->na);
					mju_copy(env->model->key_mpos + i * 3 * env->model->nmocap, env->data->mocap_pos,
					         3 * env->model->nmocap);
					mju_copy(env->model->key_mquat + i * 4 * env->model->nmocap, env->data->mocap_quat,
					         4 * env->model->nmocap);
					break;
			}
		}

		// Physics section
		else if (it && it->sectionid == SECT_PHYSICS) {
			// Update disable flags in mjOption
			env->model->opt.disableflags = 0;
			for (i = 0; i < mjNDISABLE; i++) {
				if (settings_.disable[i]) {
					env->model->opt.disableflags |= (1 << i);
				}
			}

			// Update enable flags in mjOption
			env->model->opt.enableflags = 0;
			for (i = 0; i < mjNENABLE; i++) {
				if (settings_.enable[i]) {
					env->model->opt.enableflags |= (1 << i);
				}
			}
		}

		// Rendering section
		else if (it && it->sectionid == SECT_RENDERING) {
			// Set camera in mjvCamera
			if (settings_.camera == 0) {
				free_camera_.type = mjCAMERA_FREE;
			} else if (settings_.camera == 1) {
				if (pert_.select > 0) {
					free_camera_.type        = mjCAMERA_TRACKING;
					free_camera_.trackbodyid = pert_.select;
					free_camera_.fixedcamid  = -1;
				} else {
					free_camera_.type = mjCAMERA_FREE;
					settings_.camera  = 0;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &free_context_);
				}
			} else {
				free_camera_.type       = mjCAMERA_FIXED;
				free_camera_.fixedcamid = settings_.camera - 2;
			}
			// Print floating camera as MJCF element
			if (it->itemid == 3) {
				printCamera(free_scene_.camera);
			}
		}

		// Group section
		else if (it && it->sectionid == SECT_GROUP) {
			// Remake joint section if joint group changed
			if (it->name[0] == 'J' && it->name[1] == 'o') {
				ui1_.nsect = SECT_JOINT;
				makeJoint(env, ui1_.sect[SECT_JOINT].state);
				ui1_.nsect = NSECT1;
				uiModify(main_window_, &ui1_, state, &free_context_);
			}

			// Remake control section if actuator group changed
			if (it->name[0] == 'A' && it->name[1] == 'c') {
				ui1_.nsect = SECT_CONTROL;
				makeControl(env, ui1_.sect[SECT_CONTROL].state);
				ui1_.nsect = NSECT1;
				uiModify(main_window_, &ui1_, state, &free_context_);
			}
		}

		// Stop if UI processed event
		if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Call UI 1 if event is directed to it
	if ((state->dragrect == ui1_.rectid) || (state->dragrect == 0 && state->mouserect == ui1_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// Process UI event
		mjuiItem *it = mjui_event(&ui1_, state, &free_context_);

		// control section
		if (it && it->sectionid == SECT_CONTROL) {
			// clear controls
			if (it->itemid == 0) {
				mju_zero(env->data->ctrl, env->model->nu);
				mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &free_context_);
			}
		}

		// Stop if UI processed event
		if (it != nullptr || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Short not handled by UI
	if (state->type == mjEVENT_KEY && state->key != 0) {
		switch (state->key) {
			case ' ': // Mode
				if (env->model) {
					settings_.run.store(1 - settings_.run.load());
					if (settings_.run.load())
						settings_.manual_env_steps.store(0);
					pert_.active = 0;
					mjui_update(-1, -1, &ui0_, state, &free_context_);
				}
				break;

			case mjKEY_RIGHT: // Step forward
				if (env->model && !settings_.run.load()) {
					clearTimers(env->data);
					settings_.manual_env_steps.store(1);
				}
				break;

			case mjKEY_LEFT: // Step back
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_DOWN: // Step forward 100
				if (env->model && !settings_.run.load()) {
					clearTimers(env->data);
					settings_.manual_env_steps.store(100);
				}
				break;

			case mjKEY_UP: // Step backward 100
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_PAGE_UP: // Select parent body
				if (env->model && pert_.select > 0) {
					pert_.select     = env->model->body_parentid[pert_.select];
					pert_.skinselect = -1;

					// Stop perturbation if world reached
					if (pert_.select <= 0) {
						pert_.active = 0;
					}
				}
				break;

			case '-': //  Slow down
			{
				int num_clicks = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
				if (settings_.rt_index < num_clicks - 1 && !state->shift) {
					settings_.rt_index++;
					settings_.speed_changed.store(true);
				}
			} break;

			case '=': // Speed up
				if (settings_.rt_index > 0 && !state->shift) {
					settings_.rt_index--;
					settings_.speed_changed.store(true);
				}
				break;
		}

		return;
	}

	// 3D Scroll
	if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && env->model) {
		// Emulate vertical mouse motion = 5% of window height
		mjv_moveCamera(env->model.get(), mjMOUSE_ZOOM, 0, -0.05 * state->sy, &free_scene_, &free_camera_);

		return;
	}

	// 3D press
	if (state->type == mjEVENT_PRESS && state->mouserect == 3 && env->model) {
		// Set perturbation
		int newperturb = 0;
		if (state->control && pert_.select > 0) {
			// right: translate; left: rotate
			if (state->right) {
				newperturb = mjPERT_TRANSLATE;
			} else if (state->left) {
				newperturb = mjPERT_ROTATE;
			}

			// Perturbation onset: reset reference
			if (newperturb && !pert_.active) {
				mjv_initPerturb(env->model.get(), env->data.get(), &free_scene_, &pert_);
			}
		}
		pert_.active = newperturb;

		// Handle double-click
		if (state->doubleclick) {
			// Determine selection mode
			int selmode;
			if (state->button == mjBUTTON_LEFT) {
				selmode = 1;
			} else if (state->control) {
				selmode = 3;
			} else {
				selmode = 2;
			}

			// Find geom and 3D click point, get corresponding body
			mjrRect r = state->rect[3];
			mjtNum selpnt[3];
			int selgeom, selskin;
			int selbody =
			    mjv_select(env->model.get(), env->data.get(), &vopt_, (mjtNum)r.width / (mjtNum)r.height,
			               (mjtNum)(state->x - r.left) / (mjtNum)r.width, (mjtNum)(state->y - r.bottom) / (mjtNum)r.height,
			               &free_scene_, selpnt, &selgeom, &selskin);

			// Set lookat point, start tracking is requested
			if (selmode == 2 || selmode == 3) {
				// Copy selpnt if anything clicked
				if (selbody >= 0) {
					mju_copy3(free_camera_.lookat, selpnt);
				}

				// Switch to tracking camera if dynamic body clicked
				if (selmode == 3 && selbody > 0) {
					// Mujoco camera
					free_camera_.type        = mjCAMERA_TRACKING;
					free_camera_.trackbodyid = selbody;
					free_camera_.fixedcamid  = -1;

					// UI camera
					settings_.camera = 1;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &free_context_);
				}
			}

			// Set body selection
			else {
				if (selbody >= 0) {
					// Record selection
					pert_.select     = selbody;
					pert_.skinselect = selskin;

					// Compute localpos
					mjtNum tmp[3];
					mju_sub3(tmp, selpnt, env->data->xpos + 3 * pert_.select);
					mju_mulMatTVec(pert_.localpos, env->data->xmat + 9 * pert_.select, tmp, 3, 3);
				} else {
					pert_.select     = 0;
					pert_.skinselect = 0;
				}
			}

			// Stop perturbation on select
			pert_.active = 0;
		}

		return;
	}

	// 3D release
	if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && env->model) {
		// Stop perturbation
		pert_.active = 0;

		return;
	}

	// 3D move
	if (state->type == mjEVENT_MOVE && state->dragrect == 3 && env->model) {
		// Determine action base on mouse button
		mjtMouse action;
		if (state->right) {
			action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
		} else if (state->left) {
			action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
		} else {
			action = mjMOUSE_ZOOM;
		}

		// Move perturb or camera
		mjrRect r = state->rect[3];
		if (pert_.active) {
			mjv_movePerturb(env->model.get(), env->data.get(), action, state->dx / r.height, -state->dy / r.height,
			                &free_scene_, &pert_);
		} else {
			mjv_moveCamera(env->model.get(), action, state->dx / r.height, -state->dy / r.height, &free_scene_,
			               &free_camera_);
		}

		return;
	}
}

// Prepare to render on screen
void prepareOnScreen(const ros::WallDuration &r_interval)
{
	mjtNum interval = mjMIN(1, mjMAX(0.0001, r_interval.toSec()));

	// No model: nothing to do
	if (!main_env_->model) {
		return;
	}

	// Update scene
	mjv_updateScene(main_env_->model.get(), main_env_->data.get(), &vopt_, &pert_, &free_camera_, mjCAT_ALL,
	                &free_scene_);

	// Update watch
	if (settings_.ui0 && ui0_.sect[SECT_WATCH].state) {
		watch(main_env_);
		mjui_update(SECT_WATCH, -1, &ui0_, &uistate_, &free_context_);
	}

	// Update joint
	if (settings_.ui1 && ui1_.sect[SECT_JOINT].state) {
		mjui_update(SECT_JOINT, -1, &ui1_, &uistate_, &free_context_);
	}

	// Update info text
	if (settings_.info) {
		infotext(main_env_, info_title, info_content, interval);
	}

	// Update control
	if (settings_.ui1 && ui1_.sect[SECT_CONTROL].state) {
		mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &free_context_);
	}

	// Update profiler
	if (settings_.profiler && settings_.run.load()) {
		profilerUpdate(main_env_);
	}

	// Update sensor
	if (settings_.sensor && settings_.run.load()) {
		sensorUpdate(main_env_);
	}

	// clear timers once profiler info has been copied
	clearTimers(main_env_->data);
}

void renderMain()
{
	render(main_window_);
}

} // namespace MujocoSim::rendering
