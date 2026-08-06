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
#include <mujoco_ros/util.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <mujoco/mujoco.h>
#include <mujoco_ros/array_safety.h>

#include <rosgraph_msgs/msg/clock.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp/parameter_client.hpp>

#include <mujoco_ros/description_bundle.hpp>
#include <mujoco_ros/description_converter.hpp>

#include <atomic>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <future>

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

// Monotonically increasing suffix so that concurrent kTopic resolutions
// within the same FetchRosConfiguration() call (e.g. urdf AND srdf both
// topic-sourced) never collide on either the throwaway node's name or the
// temp file's name.
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

// FetchRosConfiguration() runs inside MujocoEnv's constructor, before the
// caller has added `this` to any executor and before that executor is
// spinning -- subscribing here must never rely on `this`/`executor_` to
// service the wait, mirroring the "temporary node + throwaway executor for
// one blocking need" idiom the old cross-node param read used. Requires
// transient_local + reliable QoS on the publisher (a latched publisher):
// a volatile publisher that only sends after this subscription connects
// will never be seen within the timeout, by design -- there is no QoS
// profile that reliably receives both a pre-existing latched message and a
// live volatile stream, so this contract is fixed rather than dual-mode.
std::string ResolveTopicSource(const DescriptionSource &source, const std::string &extension)
{
	using namespace std::chrono_literals;

	auto client_node =
	    std::make_shared<rclcpp::Node>("description_bundle_topic_client_" + std::to_string(NextUniqueSuffix()));

	auto promise     = std::make_shared<std::promise<std::string>>();
	auto future      = promise->get_future();
	auto already_set = std::make_shared<std::atomic<bool>>(false);

	auto qos          = rclcpp::QoS(1).transient_local().reliable();
	auto subscription = client_node->create_subscription<std_msgs::msg::String>(
	    source.topic, qos, [promise, already_set](const std_msgs::msg::String::SharedPtr msg) {
		    if (!already_set->exchange(true)) {
			    promise->set_value(msg->data);
		    }
	    });

	auto ret = rclcpp::spin_until_future_complete(client_node, future, 5s);
	if (ret != rclcpp::FutureReturnCode::SUCCESS) {
		throw std::runtime_error(
		    "Description bundle: timed out waiting for a message on topic '" + source.topic +
		    "' (waited 5s). The publisher must publish std_msgs/String with transient_local + reliable "
		    "QoS (a latched publisher).");
	}

	return WriteContentToTempFile(future.get(), extension);
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
	RCLCPP_DEBUG(this->get_logger(), "Fetching configuration");

	rclcpp::Parameter eval_mode_param = this->get_parameter("eval_mode");
	settings_.eval_mode               = eval_mode_param.as_bool();

	rclcpp::Parameter use_sim_time_param;
	if (!this->get_parameter("use_sim_time", use_sim_time_param)) {
		RCLCPP_FATAL(this->get_logger(),
		             "/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		             "or false. Also Make sure it is set before starting any node, "
		             "otherwise nodes might behave unexpectedly.");
		throw std::runtime_error("/use_sim_time ROS param is unset.");
	}

	rclcpp::Parameter no_render_param = this->get_parameter("no_render");

	if (no_render_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "no_render is set. Disabling rendering and setting headless to true");
		this->set_parameters_atomically(
		    { rclcpp::Parameter("headless", true), rclcpp::Parameter("render_offscreen", false) });
	}

	rclcpp::Parameter render_offscreen_param = this->get_parameter("render_offscreen");
	rclcpp::Parameter headless_param         = this->get_parameter("headless");
	rclcpp::Parameter unpause_param          = this->get_parameter("unpause");
	rclcpp::Parameter num_steps_param        = this->get_parameter("num_steps");
	rclcpp::Parameter num_mj_threads_param   = this->get_parameter("num_mj_threads");

	settings_.render_offscreen = render_offscreen_param.as_bool();
	settings_.headless         = headless_param.as_bool();
	ApplyPauseState(!unpause_param.as_bool(), false);
	num_steps_until_exit_    = num_steps_param.as_int();
	settings_.num_mj_threads = num_mj_threads_param.as_int();

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
		// Empty string == unset: ROS 2 launch XML cannot conditionally omit <param>.
		if (this->has_parameter(key)) {
			const std::string value = this->get_parameter(key).as_string();
			if (!value.empty()) {
				flat_bundle_params[key] = value;
			}
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
			this->get_parameter("modelfile", modelfile);

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

			RCLCPP_INFO(this->get_logger(), "Loading model from description bundle");
			mujoco_ros::MeshPrepOptions mesh_options;
			mesh_options.convert_ascii_stl = bundle->convert_ascii_stl;
			MJR_INFO_STREAM("[Robot Description Converter][ros2-boundary] "
			                << DescribeDescriptionSource("urdf", bundle->urdf));
			if (bundle->srdf.has_value())
				MJR_INFO_STREAM("[Robot Description Converter][ros2-boundary] "
				                << DescribeDescriptionSource("srdf", *bundle->srdf));
			else
				MJR_INFO("[Robot Description Converter][ros2-boundary] srdf_source.kind=absent");
			MJR_INFO_STREAM("[Robot Description Converter][ros2-boundary] resolved_urdf_path='"
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

	std::string filename = "";

	this->get_parameter("modelfile", filename);

	rclcpp::Parameter wait_for_xml_param = this->get_parameter("wait_for_xml");
	if (wait_for_xml_param.as_bool()) {
		RCLCPP_INFO(this->get_logger(), "Waiting for mujoco_xml content parameter...");
		bool wait_for_xml = true;
		std::string xml_content;
		rclcpp::Parameter xml_content_param;
		while (wait_for_xml) {
			if (this->get_parameter("mujoco_xml", xml_content_param)) {
				RCLCPP_INFO(this->get_logger(), "Got xml content from ros parameter");
				if (!xml_content_param.get_value<std::string>().empty()) {
					filename = "rosparam_content";
				} else {
					RCLCPP_WARN(this->get_logger(), "Empty xml content received from ros parameter");
				}
				wait_for_xml = false;
			}
		}
	}

	if (!filename.empty()) {
		RCLCPP_INFO_STREAM(this->get_logger(), "Using modelfile " << filename);
		mju::strcpy_arr(queued_filename_, filename.c_str());
		RequestLoad(2);
	} else {
		RCLCPP_WARN(this->get_logger(), "No modelfile was provided, launching empty simulation!");
	}
}

void MujocoEnv::InitTFBroadcasting()
{
	static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
	tf_buffer_          = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MujocoEnv::GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type,
                                       float &pub_frequency, bool &use_segid, int &width, int &height,
                                       std::string &base_topic, std::string &rgb_topic, std::string &depth_topic,
                                       std::string &segment_topic)
{
	int stream_type_int;
	stream_type_int = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".stream_type",
	                                             static_cast<int>(rendering::kDEFAULT_CAM_STREAM_TYPE));
	stream_type     = rendering::StreamType(stream_type_int);
	pub_frequency =
	    get_maybe_undeclared_param(this, "cam_config." + cam_name + ".frequency", rendering::kDEFAULT_CAM_PUB_FREQ);
	use_segid =
	    get_maybe_undeclared_param(this, "cam_config." + cam_name + ".use_segid", rendering::kDEFAULT_CAM_USE_SEGID);
	width       = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".width", rendering::kDEFAULT_CAM_WIDTH);
	height      = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".height", rendering::kDEFAULT_CAM_HEIGHT);
	base_topic  = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".topic", "cameras/" + cam_name);
	rgb_topic   = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_rgb",
	                                         std::string(rendering::kDEFAULT_CAM_RGB_TOPIC));
	depth_topic = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_depth",
	                                         std::string(rendering::kDEFAULT_CAM_DEPTH_TOPIC));
	segment_topic = get_maybe_undeclared_param(this, "cam_config." + cam_name + ".name_segment",
	                                           std::string(rendering::kDEFAULT_CAM_SEGMENT_TOPIC));
}

void MujocoEnv::GetInitialJointPositions(std::map<std::string, std::vector<double>> &joint_pos_map)
{
	std::string param_name = "initial_joint_states";
	auto result            = this->list_parameters({ param_name }, 2);

	if (result.names.empty()) {
		RCLCPP_WARN(this->get_logger(),
		            "No initial joint position specified (failed to get 'initial_joint_states' parameter).");
		return;
	}
	std::vector<std::string> joint_names;

	for (const auto &joint_name : result.names) {
		if (joint_name.rfind(param_name + ".", 0) == 0) {
			joint_names.push_back(joint_name);
		}
	}

	auto parameters = this->get_parameters(joint_names);

	for (const auto &joint : parameters) {
		std::string joint_name = joint.get_name().substr(param_name.length() + 1); // Strip "initial_joint_states."
		if (joint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
			joint_pos_map[joint_name] = joint.as_double_array();
			RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << joint_name << " has initial values: ["
			                                                << mujoco_ros::util::vector_to_string(joint.as_double_array())
			                                                << "]");
		} else {
			RCLCPP_WARN(this->get_logger(), "Joint %s is not a double array", joint.get_name().c_str());
		}
	}
}

void MujocoEnv::GetInitialJointVelocities(std::map<std::string, std::vector<double>> &joint_vel_map)
{
	std::string param_name = "initial_joint_velocities";
	auto result            = this->list_parameters({ param_name }, 2);

	if (result.names.empty()) {
		RCLCPP_WARN(this->get_logger(),
		            "No initial joint velocity specified (failed to get 'initial_joint_velocities' parameter).");
		return;
	}
	std::vector<std::string> joint_names;

	for (const auto &joint_name : result.names) {
		if (joint_name.rfind(param_name + ".", 0) == 0) {
			joint_names.push_back(joint_name);
		}
	}

	auto parameters = this->get_parameters(joint_names);

	for (const auto &joint : parameters) {
		std::string joint_name = joint.get_name().substr(param_name.length() + 1); // Strip "initial_joint_velocities."
		if (joint.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
			joint_vel_map[joint_name] = joint.as_double_array();
			RCLCPP_INFO_STREAM(this->get_logger(), "Joint " << joint_name << " has initial velocity values: ["
			                                                << mujoco_ros::util::vector_to_string(joint.as_double_array())
			                                                << "]");
		} else {
			RCLCPP_WARN(this->get_logger(), "Joint %s is not a double array", joint.get_name().c_str());
		}
	}
}

// TODO:
// setupServices
// callbacks

} // namespace mujoco_ros
