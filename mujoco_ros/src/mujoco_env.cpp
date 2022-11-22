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
#include <mujoco_ros/render_utils.h>

#include <sstream>

namespace MujocoSim {

namespace environments {

void assignData(mjData *data, MujocoEnvPtr env)
{
	env->data.reset(data);
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

MujocoEnvParallel::MujocoEnvParallel(const std::string &ros_ns, const std::string &launchfile,
                                     const std::vector<std::string> &launch_args)
    : MujocoEnv(ros_ns), launchfile(launchfile), launch_args(launch_args)
{
	if (!launchfile.empty())
		bootstrapNamespace();
}

void MujocoEnvParallel::bootstrapNamespace()
{
	mujoco_ros_msgs::BootstrapNS bootstrap_msg;
	bootstrap_msg.request.ros_namespace = name;
	bootstrap_msg.request.launchfile    = launchfile;
	bootstrap_msg.request.args          = launch_args;

	if (!ros::service::waitForService("/bootstrap_ns", ros::Duration(5))) {
		ROS_ERROR_NAMED("mujoco_env",
		                "Timeout while waiting for namespace bootstrapping node under topic '/bootstrap_ns'. "
		                "Is it started correctly?");
		return;
	}

	if (!ros::service::call("/bootstrap_ns", bootstrap_msg))
		ROS_ERROR_STREAM_NAMED("mujoco_env", "Error while bootstrapping ROS environment for namespace '" << name << "'");
}

void MujocoEnv::initializeRenderResources()
{
	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Initializing offscreen rendering utils [" << name << "]");
	if (vis.window != NULL) {
		ROS_DEBUG_NAMED("mujoco_env", "\tDestroying old offscreen buffer window");
		glfwDestroyWindow(vis.window);
	}

	ROS_DEBUG_NAMED("mujoco_env", "\tCreating new offscreen buffer window");
	glfwWindowHint(GLFW_DOUBLEBUFFER, 0);
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	vis.window = glfwCreateWindow(800, 800, ("invisible window " + name).c_str(), nullptr, nullptr);
	if (!vis.window) {
		ROS_ERROR_NAMED("mujoco_env", "Could not create GLFW window!");
		mju_error("Could not create GLFW window");
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
	mjv_makeScene(model.get(), &(vis.scn), render_utils::maxgeom_);
	ROS_DEBUG_NAMED("mujoco_env", "\tApplied model to scene");
	mjr_makeContext(model.get(), &(vis.con), 50 * (settings_.font + 1));
	ROS_DEBUG_NAMED("mujoco_env", "\tApplied model to context");
	vis.viewport = mjr_maxViewport(&(vis.con));
	ROS_DEBUG_NAMED("mujoco_env", "\tComputed context viewport");

	vis.rgb.reset(new unsigned char[vis.viewport.width * vis.viewport.height * 3],
	              std::default_delete<unsigned char[]>());
	vis.depth.reset(new float[vis.viewport.width * vis.viewport.height], std::default_delete<float[]>());

	image_transport::ImageTransport it(*nh);
	bool config_exists, use_segid;
	render_utils::streamType stream_type;
	std::string cam_name, cam_config_path;
	double pub_freq;

	// // TODO(dleins): move camera pub config to URDF/SRDF config once it's ready
	config_exists = nh->searchParam("cam_config", cam_config_path) || ros::param::search("cam_config", cam_config_path);
	ROS_DEBUG_STREAM_COND_NAMED(config_exists, "mujoco_env", "Found camera config under path: " << cam_config_path);

	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Model has " << model->ncam << " cameras");

	render_utils::CamStreamPtr stream_ptr;
	for (int cam_id = 0; cam_id < model->ncam; cam_id++) {
		cam_name = mj_id2name(model.get(), mjOBJ_CAMERA, cam_id);
		ROS_DEBUG_STREAM_NAMED("mujoco_env",
		                       "Found camera '" << cam_name << "' with id " << cam_id << ". Setting up publishers...");
		if (config_exists) {
			stream_type = render_utils::streamType(
			    nh->param<int>(cam_config_path + "/" + cam_name + "/stream_type", render_utils::streamType::RGB));
			pub_freq  = nh->param<float>(cam_config_path + "/" + cam_name + "/frequency", 15);
			use_segid = nh->param<bool>(cam_config_path + "/" + cam_name + "/use_segid", true);
		} else {
			ROS_DEBUG_NAMED("mujoco_env", "\tUsing camera stream default config");
			stream_type = render_utils::streamType::RGB;
			pub_freq    = 15;
			use_segid   = true;
		}

		image_transport::Publisher rgb, depth, segment;
		if (stream_type & render_utils::streamType::RGB) {
			ROS_DEBUG_NAMED("mujoco_env", "\tCreating rgb publisher");
			rgb = it.advertise("cameras/" + cam_name + "/rgb", 1);
		}
		if (stream_type & render_utils::streamType::DEPTH) {
			ROS_DEBUG_NAMED("mujoco_env", "\tCreating depth publisher");
			depth = it.advertise("cameras/" + cam_name + "/depth", 1);
		}
		if (stream_type & render_utils::streamType::SEGMENTED) {
			ROS_DEBUG_NAMED("mujoco_env", "\tCreating segmentation publisher");
			segment = it.advertise("cameras/" + cam_name + "/segmented", 1);
		}

		ROS_DEBUG_STREAM_NAMED("mujoco_env", "\tSetting up camera stream(s) of type '"
		                                         << stream_type << "' with a publish rate of " << pub_freq
		                                         << " Hz for camera named " << cam_name);

		stream_ptr.reset(new render_utils::CamStream(cam_id, stream_type, rgb, depth, segment, use_segid, pub_freq));
		cam_streams.push_back(stream_ptr);
	}
}

void MujocoEnv::reload()
{
	ROS_DEBUG_STREAM_NAMED("mujoco_env", "(re)loading MujocoPlugins ... [" << name << "]");
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
	ROS_DEBUG_STREAM_NAMED("mujoco_env", "Done (re)loading MujocoPlugins [" << name << "]");
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

void MujocoEnv::runRenderCbs()
{
	if (plugins.empty())
		return;
	for (const auto &plugin : cb_ready_plugins) {
		plugin->renderCallback(model, data, &(vis.scn));
	}
}

void MujocoEnv::runLastStageCbs()
{
	for (const auto &plugin : cb_ready_plugins) {
		plugin->lastStageCallback(model, data);
	}
}

MujocoEnv::~MujocoEnv()
{
	cb_ready_plugins.clear();
	plugins.clear();
	model.reset();
	data.reset();
	vis.rgb.reset();
	vis.depth.reset();
	mjr_freeContext(&(vis.con));
	mjv_freeScene(&(vis.scn));
	ctrlnoise = nullptr;
	nh.reset();
}

MujocoEnvParallel::~MujocoEnvParallel()
{
	mujoco_ros_msgs::ShutdownNS shutdown_msg;
	shutdown_msg.request.ros_namespace = name;

	ros::service::call("/shutdown_ns", shutdown_msg);
}

} // namespace MujocoSim
