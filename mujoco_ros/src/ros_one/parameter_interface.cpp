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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <ros/ros.h>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_one/plugin_utils.hpp>

#include <mujoco/mujoco.h>
#include <mujoco_ros/array_safety.h>

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/String.h>

#include <mujoco_ros/description_bundle.hpp>
#include <mujoco_ros/description_converter.hpp>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace mju = ::mujoco::sample_util;

namespace mujoco_ros {

namespace {

const char *DescriptionSourceKindName(const mujoco_ros::DescriptionSource &source)
{
	switch (source.kind) {
		case mujoco_ros::DescriptionSource::Kind::kFile:
			return "file";
		case mujoco_ros::DescriptionSource::Kind::kTopic:
			return "topic";
	}
	return "unknown";
}

std::string DescribeDescriptionSource(const char *label, const mujoco_ros::DescriptionSource &source)
{
	std::ostringstream out;
	out << label << "_source.kind=" << DescriptionSourceKindName(source);
	if (source.kind == mujoco_ros::DescriptionSource::Kind::kFile)
		out << " " << label << "_source.path='" << source.path << "'";
	else
		out << " " << label << "_source.topic='" << source.topic << "'";
	return out.str();
}

// Monotonically increasing suffix so that resolving urdf AND srdf as
// kTopic sources within the same FetchRosConfiguration() call never
// collide on the temp file name.
uint64_t NextUniqueSuffix()
{
	static std::atomic<uint64_t> counter{ 0 };
	return counter.fetch_add(1);
}

std::string WriteContentToTempFile(const std::string &content, const std::string &extension)
{
	std::string tmp_path = (std::filesystem::temp_directory_path() /
	                        ("mujoco_ros_description_bundle_" + std::to_string(NextUniqueSuffix()) + extension))
	                           .string();
	std::ofstream out(tmp_path, std::ios::trunc);
	if (!out.is_open()) {
		throw std::runtime_error("Description bundle: failed to create temp file '" + tmp_path + "'");
	}
	out << content;
	out.close();
	return tmp_path;
}

// The main() AsyncSpinner (mujoco_ros/src/main.cpp) is already running
// before MujocoEnv is constructed, so subscriber callbacks fire during this
// wait without any local spin call -- unlike ROS 2, there is no "not
// spinning yet" hazard to mitigate here. Requires latch=true on the
// publisher: a non-latched publisher that only sends after this subscriber
// connects will never be seen within the timeout, by design -- see
// ResolveTopicSource in ros_two/parameter_interface.cpp for the equivalent
// ROS 2 contract.
std::string ResolveTopicSource(const DescriptionSource &source, const std::string &extension)
{
	std::string content;
	std::atomic<bool> received{ false };

	ros::NodeHandle nh;
	ros::Subscriber sub =
	    nh.subscribe<std_msgs::String>(source.topic, 1,
	                                   boost::function<void(const std_msgs::String::ConstPtr &)>(
	                                       [&content, &received](const std_msgs::String::ConstPtr &msg) {
		                                       if (!received.exchange(true)) {
			                                       content = msg->data;
		                                       }
	                                       }));

	const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(5.0);
	while (!received.load() && ros::WallTime::now() < deadline) {
		ros::WallDuration(0.05).sleep();
	}

	if (!received.load()) {
		throw std::runtime_error("Description bundle: timed out waiting for a message on topic '" + source.topic +
		                         "' (waited 5s). The publisher must latch (publish with latch=true).");
	}

	return WriteContentToTempFile(content, extension);
}

// kFile sources resolve directly to their configured path; kTopic sources
// read the content over a subscription and stash it in a fresh temp file
// (tracked in temp_files_to_remove so the caller can clean it up once
// load_model_from_description() is done with it -- unlike the temp .mjb
// SaveDescriptionToTempMjb produces internally, these input files are not
// cleaned up by that function itself).
std::string ResolveDescriptionSourcePath(const DescriptionSource &source, const std::string &extension,
                                         std::vector<std::string> &temp_files_to_remove)
{
	if (source.kind == DescriptionSource::Kind::kFile) {
		return source.path;
	}
	std::string tmp_path = ResolveTopicSource(source, extension);
	temp_files_to_remove.push_back(tmp_path);
	return tmp_path;
}

} // namespace

void MujocoEnv::FetchRosConfiguration()
{
	ROS_DEBUG("Fetching configuration");

	bool eval_mode = false;
	nh_->param("eval_mode", eval_mode, false);
	settings_.eval_mode = eval_mode;
	if (!ros::param::get("/use_sim_time", settings_.use_sim_time)) {
		ROS_FATAL("/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		          "or false. Also Make sure it is set before starting any node, "
		          "otherwise nodes might behave unexpectedly.");
		throw std::runtime_error("/use_sim_time ROS param is unset.");
	}
	bool no_render;
	nh_->param("no_render", no_render, false);
	nh_->param("render_offscreen", settings_.render_offscreen, true);
	nh_->param("headless", settings_.headless, true);

	if (no_render) {
		ROS_INFO("no_render is set. Disabling rendering and setting headless to true");
		nh_->setParam("headless", true);
		nh_->setParam("render_offscreen", false);

		settings_.headless         = true;
		settings_.render_offscreen = false;
	}

	nh_->param("num_steps", num_steps_until_exit_, -1);
	nh_->param("num_mj_threads", settings_.num_mj_threads, settings_.num_mj_threads);
	bool run;
	nh_->param("unpause", run, true);
	ApplyPauseState(!run, false);

	// Description bundle: additive to the modelfile path below -- activation
	// is "urdf.source" alone ("srdf.source" is optional; absent means no
	// disable_collisions). If "urdf.source" isn't configured,
	// TryParseDescriptionBundleFromMap returns std::nullopt and this falls
	// through unchanged to the pre-existing modelfile-driven load. With a
	// bundle active, "modelfile" changes meaning: it is the world MJCF to
	// compose the robot into (empty -> LoadDefaultWorldSpec()'s built-in
	// default world), never a robot/world combo to parse standalone.
	std::map<std::string, std::string> flat_bundle_params;
	for (const char *key :
	     { "urdf.source", "urdf.path", "urdf.topic", "srdf.source", "srdf.path", "srdf.topic",
	       "description.convert_ascii_stl", "description.generate_actuators", "description.attach_prefix" }) {
		std::string value;
		// Empty string == unset (matches ROS 2 launch always-pass + skip-empty).
		if (nh_->hasParam(key) && nh_->getParam(key, value) && !value.empty()) {
			flat_bundle_params[key] = value;
		}
	}

	auto bundle = mujoco_ros::TryParseDescriptionBundleFromMap(flat_bundle_params);
	if (bundle.has_value()) {
		std::vector<std::string> temp_files_to_remove;
		mjSpec *owned_world = nullptr;
		try {
			std::string urdf_path = ResolveDescriptionSourcePath(bundle->urdf, ".urdf", temp_files_to_remove);
			std::string srdf_path;
			if (bundle->srdf.has_value()) {
				srdf_path = ResolveDescriptionSourcePath(*bundle->srdf, ".srdf", temp_files_to_remove);
			}

			std::string modelfile;
			nh_->getParam("modelfile", modelfile);

			mjSpec *world_spec = nullptr;
			if (!modelfile.empty()) {
				char error[1000] = { 0 };
				owned_world      = mj_parseXML(modelfile.c_str(), nullptr, error, sizeof(error));
				if (owned_world == nullptr) {
					throw std::runtime_error("Description bundle: failed to parse modelfile as world '" + modelfile +
					                         "': " + error);
				}
				world_spec = owned_world;
			}

			ROS_INFO("Loading model from description bundle");
			mujoco_ros::MeshPrepOptions mesh_options;
			mesh_options.convert_ascii_stl = bundle->convert_ascii_stl;
			MJR_INFO_STREAM("[Robot Description Converter][ros1-boundary] "
			                << DescribeDescriptionSource("urdf", bundle->urdf));
			if (bundle->srdf.has_value())
				MJR_INFO_STREAM("[Robot Description Converter][ros1-boundary] "
				                << DescribeDescriptionSource("srdf", *bundle->srdf));
			else
				MJR_INFO("[Robot Description Converter][ros1-boundary] srdf_source.kind=absent");
			MJR_INFO_STREAM("[Robot Description Converter][ros1-boundary] resolved_urdf_path='"
			                << urdf_path << "' resolved_srdf_path='" << srdf_path << "' modelfile='" << modelfile
			                << "' generate_actuators=" << std::boolalpha << bundle->generate_actuators
			                << " attach_prefix='" << bundle->attach_prefix << "'");
			auto [model, data] = mujoco_ros::load_model_from_description(
			    urdf_path, srdf_path, world_spec, mesh_options, bundle->generate_actuators, bundle->attach_prefix);

			if (owned_world != nullptr) {
				mj_deleteSpec(owned_world);
				owned_world = nullptr;
			}
			for (const auto &tmp_path : temp_files_to_remove) {
				std::remove(tmp_path.c_str());
			}

			// Same already-loaded-model/data-pair load route LoadPythonModel
			// (python/bindings/src/mujoco_env.cpp) uses -- not queued_filename_/load_request=2,
			// which is for the *other* load route (InitModelFromQueue() parsing from a filename).
			// Unlike LoadPythonModel, is_python_request stays 0: that flag tells
			// LoadWithModelAndData() the model/data buffers are Python-owned and must get a
			// no-op deleter instead of mj_deleteModel/mj_deleteData. load_model_from_description()
			// allocates via plain mj_loadModel/mj_makeData -- genuinely C++-owned -- so setting it
			// here would permanently leak this model on every subsequent reload/destruction.
			// Locking physics_thread_mutex_ here is uncontended (no physics thread exists yet at
			// this point in the constructor) but kept for consistency with LoadPythonModel's call site.
			RecursiveLock lock(physics_thread_mutex_);
			mnew = model;
			dnew = data;
			mju::strcpy_arr(filename_, urdf_path.c_str());
			settings_.is_python_request.store(0);
			RequestLoad(1);
		} catch (...) {
			if (owned_world != nullptr)
				mj_deleteSpec(owned_world);
			for (const auto &tmp_path : temp_files_to_remove) {
				std::remove(tmp_path.c_str());
			}
			throw;
		}
		return;
	}

	/*
	 * Model (file) passing: the model can be provided as file to parse or directly as string stored in the rosparam
	 * server. If both string and file are provided, the string takes precedence.
	 */
	std::string filename = "";
	nh_->getParam("modelfile", filename);
	std::string xml_content_path;
	std::string xml_content;

	bool wait_for_xml;
	nh_->param("wait_for_xml", wait_for_xml, false);

	ROS_INFO_COND(wait_for_xml, "Waiting for xml content to be available on rosparam server");

	// TODO: add timeout of 60 seconds
	while (wait_for_xml) {
		if (nh_->searchParam("mujoco_xml", xml_content_path) || ros::param::search("mujoco_xml", xml_content_path)) {
			ROS_DEBUG_STREAM("Found mujoco_xml_content param under " << xml_content_path);

			nh_->getParam(xml_content_path, xml_content);
			if (!xml_content.empty()) {
				ROS_INFO("Got xml content from ros param server");
				ROS_WARN_COND(!filename.empty(),
				              "Both xml content and modelfile supplied as parameters! Using xml content.");
				filename = "rosparam_content";
			}
			wait_for_xml = false;
		}
	}

	ROS_INFO_STREAM_COND(!filename.empty(), "Using modelfile " << filename);
	ROS_WARN_COND(filename.empty(), "No modelfile was provided, launching empty simulation!");

	if (!filename.empty()) {
		mju::strcpy_arr(queued_filename_, filename.c_str());
		RequestLoad(2);
	}
}

void MujocoEnv::InitTFBroadcasting()
{
	static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
	tf_buffer_          = std::make_unique<tf2_ros::Buffer>();
	tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MujocoEnv::GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type,
                                       float &pub_frequency, bool &use_segid, int &width, int &height,
                                       std::string &base_topic, std::string &rgb_topic, std::string &depth_topic,
                                       std::string &segment_topic)
{
	bool config_exists;
	std::string cam_config_path;
	config_exists = nh_->searchParam("cam_config", cam_config_path) || ros::param::search("cam_config", cam_config_path);
	MJR_DEBUG_STREAM_COND(config_exists, "Found camera config under path: " << cam_config_path);

	stream_type = rendering::StreamType(
	    nh_->param<int>(cam_config_path + "/" + cam_name + "/stream_type", rendering::kDEFAULT_CAM_STREAM_TYPE));
	pub_frequency = nh_->param<float>(cam_config_path + "/" + cam_name + "/frequency", rendering::kDEFAULT_CAM_PUB_FREQ);
	use_segid     = nh_->param<bool>(cam_config_path + "/" + cam_name + "/use_segid", rendering::kDEFAULT_CAM_USE_SEGID);
	width         = nh_->param<int>(cam_config_path + "/" + cam_name + "/width", rendering::kDEFAULT_CAM_WIDTH);
	height        = nh_->param<int>(cam_config_path + "/" + cam_name + "/height", rendering::kDEFAULT_CAM_HEIGHT);
	base_topic    = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/topic", "cameras/" + cam_name);
	rgb_topic     = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_rgb",
	                                        std::string(rendering::kDEFAULT_CAM_RGB_TOPIC));
	depth_topic   = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_depth",
	                                        std::string(rendering::kDEFAULT_CAM_DEPTH_TOPIC));
	segment_topic = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_segment",
	                                        std::string(rendering::kDEFAULT_CAM_SEGMENT_TOPIC));
}

void MujocoEnv::GetInitialJointPositions(std::map<std::string, std::vector<double>> &joint_pos_map)
{
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_positions/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_positions/joint_map") && joint_map.empty()) {
		MJR_WARN("Initial joint positions not recognized by rosparam server. Check your config, "
		         "especially values for single axis joints should explicitly provided as string!");
		return;
	}

	for (auto const &[name, str_values] : joint_map) {
		MJR_DEBUG_STREAM("fetched jointpos values of joint " << name << ": " << str_values);

		std::vector<double> axis_vals;
		axis_vals.reserve(7);

		std::stringstream stream_values(str_values);
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			axis_vals.push_back(std::stod(value));
		}
		axis_vals.shrink_to_fit();
		joint_pos_map[name] = axis_vals;
	}
}

void MujocoEnv::GetInitialJointVelocities(std::map<std::string, std::vector<double>> &joint_vel_map)
{
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_velocities/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_velocities/joint_map") && joint_map.empty()) {
		MJR_WARN("Initial joint velocities not recognized by rosparam server. Check your config, "
		         "especially values for single axis joints should explicitly provided as string!");
		return;
	}

	for (auto const &[name, str_values] : joint_map) {
		MJR_DEBUG_STREAM("fetched jointvel values of joint " << name << ": " << str_values);

		std::vector<double> axis_vals;
		axis_vals.reserve(7);

		std::stringstream stream_values(str_values);
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			axis_vals.push_back(std::stod(value));
		}
		axis_vals.shrink_to_fit();
		joint_vel_map[name] = axis_vals;
	}
}

// TODO:
// setupServices
// callbacks

} // namespace mujoco_ros
