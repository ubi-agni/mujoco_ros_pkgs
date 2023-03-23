/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <ros/ros.h>

#include <mujoco_ros_msgs/BootstrapNS.h>
#include <mujoco_ros_msgs/ShutdownNS.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/rendering/utils.h>
#include <mujoco_ros/rendering/camera.h>

#include <sstream>

namespace MujocoSim {

namespace environments {

void assignData(mjData *data, MujocoEnvPtr env)
{
	env->data.reset(data, [](mjData *d) { mj_deleteData(d); });
	env_map_[data] = env;
}

MujocoEnvPtr getEnv(mjData *data)
{
	if (env_map_.find(data) != env_map_.end()) {
		return env_map_[data];
	} else
		return nullptr;
}

MujocoEnvPtr getEnvById(uint id)
{
	for (std::pair<mjData *, MujocoEnvPtr> element : env_map_) {
		if (element.second->name == "/env" + id || (element.second->name == "/" && id == 0)) {
			return element.second;
		}
	}
	return nullptr;
}

void unregisterEnv(mjData *data)
{
	if (env_map_.find(data) != env_map_.end()) {
		env_map_.erase(data);
	}
}

void unregisterEnv(uint id)
{
	for (std::pair<mjData *, MujocoEnvPtr> element : env_map_) {
		if (element.second->name == "/env" + id || (element.second->name == "/" && id == 0)) {
			env_map_.erase(element.second->data.get());
		}
	}
}

} // end namespace environments

void MujocoEnv::initializeRenderResources()
{
	image_transport::ImageTransport it(*nh);
	bool config_exists, use_segid;
	rendering::streamType stream_type;
	std::string cam_name, cam_config_path;
	double pub_freq;
	int max_res_h = 0, max_res_w = 0;

	// // TODO(dleins): move camera pub config to URDF/SRDF config once it's ready
	config_exists = nh->searchParam("cam_config", cam_config_path) || ros::param::search("cam_config", cam_config_path);
	ROS_DEBUG_STREAM_COND_NAMED(config_exists, "mujoco_env", "Found camera config under path: " << cam_config_path);

	if (model->ncam == 0) {
		ROS_DEBUG_NAMED("mujoco_env", "Model has no cameras, skipping offscreen render utils init");
		return;
	}

	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Model has " << model->ncam << " cameras");

	cam_streams.clear();

	rendering::CameraStreamPtr stream_ptr;
	int res_h, res_w;
	for (int cam_id = 0; cam_id < model->ncam; cam_id++) {
		cam_name = mj_id2name(model.get(), mjOBJ_CAMERA, cam_id);
		ROS_DEBUG_STREAM_NAMED("mujoco_env",
		                       "Found camera '" << cam_name << "' with id " << cam_id << ". Setting up publishers...");

		stream_type = rendering::streamType(
		    nh->param<int>(cam_config_path + "/" + cam_name + "/stream_type", rendering::streamType::RGB));
		pub_freq  = nh->param<float>(cam_config_path + "/" + cam_name + "/frequency", 15);
		use_segid = nh->param<bool>(cam_config_path + "/" + cam_name + "/use_segid", true);
		res_w     = nh->param<int>(cam_config_path + "/" + cam_name + "/width", 720);
		res_h     = nh->param<int>(cam_config_path + "/" + cam_name + "/height", 480);

		max_res_h = std::max(res_h, max_res_h);
		max_res_w = std::max(res_w, max_res_w);

		stream_ptr.reset(new rendering::CameraStream(cam_id, cam_name, res_w, res_h, stream_type, use_segid, pub_freq,
		                                             &it, nh, model, data));
		cam_streams.push_back(stream_ptr);
	}

	if (model->vis.global.offheight < max_res_h || model->vis.global.offwidth < max_res_w) {
		ROS_WARN_STREAM_NAMED("mujoco_env", "Model offscreen resolution too small for configured cameras, updating "
		                                    "offscreen resolution to fit cam config ... ("
		                                        << max_res_w << "x" << max_res_h << ")");
		model->vis.global.offheight = max_res_h;
		model->vis.global.offwidth  = max_res_w;
	}

	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Initializing offscreen rendering utils [" << name << "]");
	if (vis.window == nullptr) {
		ROS_DEBUG_NAMED("mujoco_env", "\tCreating new offscreen buffer window");
		glfwWindowHint(GLFW_DOUBLEBUFFER, 0);
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
		vis.window = glfwCreateWindow(max_res_h, max_res_w, ("invisible window " + name).c_str(), nullptr, nullptr);
		if (!vis.window) {
			ROS_ERROR_NAMED("mujoco_env", "Could not create GLFW window!");
			mju_error("Could not create GLFW window");
		}
	} else {
		ROS_DEBUG_NAMED("mujoco_env", "Reusing old offscreen buffer window");
	}

	// make context current
	glfwMakeContextCurrent(vis.window);
	glfwSwapInterval(0);

	// init vis data structure
	mjv_defaultCamera(&(vis.cam));
	ROS_DEBUG_NAMED("mujoco_env", "\tInitialized camera");
	mjv_defaultOption(&(vis.vopt));
	ROS_DEBUG_NAMED("mujoco_env", "\tInitialized v-options");
	mjv_defaultScene(&(vis.scn));
	ROS_DEBUG_NAMED("mujoco_env", "\tInitialized default scene");
	mjr_defaultContext(&(vis.con));
	ROS_DEBUG_NAMED("mujoco_env", "\tInitialized default context");

	// // Create scene and context
	mjv_makeScene(model.get(), &(vis.scn), rendering::maxgeom_);
	ROS_DEBUG_NAMED("mujoco_env", "\tApplied model to scene");
	mjr_makeContext(model.get(), &(vis.con), 50 * (settings_.font + 1));
	ROS_DEBUG_NAMED("mujoco_env", "\tApplied model to context");

	int buffer_size = max_res_w * max_res_h;
	vis.rgb.reset(new unsigned char[buffer_size * 3], std::default_delete<unsigned char[]>());
	vis.depth.reset(new float[buffer_size], std::default_delete<float[]>());
}

void MujocoEnv::load()
{
	ROS_DEBUG_STREAM_NAMED("mujoco_env", "loading MujocoPlugins ... [" << name << "]");
	cb_ready_plugins.clear();
	plugins.clear();

	XmlRpc::XmlRpcValue plugin_config;
	if (plugin_utils::parsePlugins(nh, plugin_config)) {
		plugin_utils::registerPlugins(nh, plugin_config, plugins);
	}

	for (const auto &plugin : plugins) {
		if (plugin->safe_load(model, data)) {
			cb_ready_plugins.push_back(plugin);
		}
	}
	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Done loading MujocoPlugins [" << name << "]");
}

void MujocoEnv::reset()
{
	for (const auto &plugin : plugins) {
		plugin->safe_reset();
	}
}

const std::vector<MujocoPluginPtr> MujocoEnv::getPlugins()
{
	return plugins;
}

void MujocoEnv::runControlCbs()
{
	for (const auto &plugin : cb_ready_plugins) {
		plugin->controlCallback(model, data);
	}
}

void MujocoEnv::runPassiveCbs()
{
	for (const auto &plugin : cb_ready_plugins) {
		plugin->passiveCallback(model, data);
	}
}

void MujocoEnv::runRenderCbs(mjvScene *scene)
{
	if (plugins.empty())
		return;
	for (const auto &plugin : cb_ready_plugins) {
		plugin->renderCallback(model, data, scene);
	}
}

void MujocoEnv::runLastStageCbs()
{
	for (const auto &plugin : cb_ready_plugins) {
		plugin->lastStageCallback(model, data);
	}
}

void MujocoEnv::notifyGeomChanged(const int geom_id)
{
	for (const auto &plugin : cb_ready_plugins) {
		plugin->onGeomChanged(model, data, geom_id);
	}
}

void MujocoEnv::prepareReload()
{
	model.reset();
	data.reset();
	vis.rgb.reset();
	vis.depth.reset();
	cb_ready_plugins.clear();
	plugins.clear();
	cam_streams.clear();
}

MujocoEnv::~MujocoEnv()
{
	free(ctrlnoise);
	cb_ready_plugins.clear();
	plugins.clear();
	model.reset();
	data.reset();
	vis.rgb.reset();
	vis.depth.reset();
	nh.reset();
}

} // namespace MujocoSim
