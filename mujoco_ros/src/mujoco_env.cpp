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

#include <stdexcept>
#include <sstream>

#include <mujoco/mujoco.h>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/viewer.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>

#include <geometry_msgs/TransformStamped.h>
using TransformStamped = geometry_msgs::TransformStamped;

namespace roscpp = ros;
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace roscpp                  = rclcpp;
#endif

#if RENDER_BACKEND == GLFW_BACKEND
static std::string render_backend = "GLFW";
#elif RENDER_BACKEND == OSMESA_BACKEND
static std::string render_backend = "OSMesa";
#elif RENDER_BACKEND == EGL_BACKEND
static std::string render_backend = "EGL";
#else
static std::string render_backend = "NONE. No offscreen rendering available.";
#endif

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

#if RENDER_BACKEND == GLFW_BACKEND
namespace {
int MaybeGlfwInit()
{
	static const int is_initialized = []() {
		auto success = Glfw().glfwInit();
		if (success == GLFW_TRUE) {
			std::atexit(Glfw().glfwTerminate);
		} else {
			const char *description;
			int error = glfwGetError(&description);
			MJR_ERROR("Failed to initialize GLFW: %d %s", error, description);
		}
		return success;
	}();
	return is_initialized;
}
} // namespace
#endif

MujocoEnv *MujocoEnv::instance = nullptr;

void MujocoEnv::RunRenderCbs(mjvScene * /*scene*/) {}
void UpdateModelFlags(const mjOption *) {}
void MujocoEnv::RunLastStageCbs() {
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->WrappedLastStageCallback(this->model_.get(), this->data_.get());
	}
}

#if MJR_ROS_VERSION == ROS_1

MujocoEnv::MujocoEnv(const std::string &admin_hash /* = std::string()*/)
{
	if (!admin_hash.empty()) {
		mju::strcpy_arr(settings_.admin_hash, admin_hash.c_str());
	} else {
		// make sure hash is empty and null-terminated
		settings_.admin_hash[0] = '\0';
	}

	nh_      = std::make_shared<ros::NodeHandle>("~");
	ros_api_ = std::make_unique<RosAPI>(nh_, this);
	plugin_utils::InitPluginLoader();
	Configure();
}

#else // MJR_ROS_VERSION == ROS_2

MujocoEnv::MujocoEnv(rclcpp::Executor::SharedPtr executor, const std::string &admin_hash /* = std::string()*/)
    : rclcpp::Node("mujoco_server", "", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    , executor_(executor)
{
	if (!admin_hash.empty()) {
		mju::strcpy_arr(settings_.admin_hash, admin_hash.c_str());
	} else {
		// make sure hash is empty and null-terminated
		settings_.admin_hash[0] = '\0';
	}

	ros_api_ = std::make_unique<RosAPI>(this);
	plugin_utils::InitPluginLoader();
	Configure();
}

void MujocoEnv::AddNodeToExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
	executor_->add_node(node);
}

void MujocoEnv::RemoveNodeFromExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
	executor_->remove_node(node);
}

rclcpp::Executor::SharedPtr MujocoEnv::GetExecutorPtr(){
	return executor_;
}

#endif

void MujocoEnv::Configure()
{
	MJR_DEBUG("Configuring simulation server");

	FetchRosConfiguration();
	ros_api_->SetupServices();

	if (settings_.eval_mode) {
		MJR_INFO("Running in evaluation mode. Parsing admin hash...");
		if (!settings_.admin_hash[0]) {
			MJR_ERROR("Evaluation mode requires a hash to verify critical operations are allowed. No hash was provided, "
			          "aborting launch.");
			settings_.exit_request = 1;
			throw std::runtime_error(
			    "Evaluation mode requires a hash to verify critical operations are allowed. No hash was "
			    "provided, aborting launch.");
		}
	}

	MJR_DEBUG_COND(!settings_.use_sim_time, "use_sim_time is set to false. Not publishing sim time to /clock!");

	MJR_INFO_STREAM("Using MuJoCo library version " << mj_versionString());
	if (mjVERSION_HEADER != mj_version()) {
		MJR_WARN_STREAM("Headers and library have different versions (headers: " << mjVERSION_HEADER
		                                                                         << ", library: " << mj_version() << ")");
	}
	MJR_INFO_STREAM("Compiled with render backend: " << render_backend);

	if (!settings_.headless) {
#if RENDER_BACKEND == GLFW_BACKEND
		gui_adapter_ = new mujoco_ros::GlfwAdapter();
#else
		MJR_ERROR("Compiled without GLFW support. Cannot run in non-headless mode.");
#endif
	}
	ros_api_->SetupClockPublisher();

	MJR_INFO_COND(!settings_.run, "Starting Simulation in paused mode");

	mjv_defaultScene(&scn_);
	mjv_defaultPerturb(&pert_);

	if (settings_.render_offscreen) {
		bool can_render = true;

#if RENDER_BACKEND == GLFW_BACKEND
		can_render = MaybeGlfwInit();
		MJR_ERROR_COND(!can_render, "Failed to initialize GLFW. Cannot render offscreen!");
#elif RENDER_BACKEND == NO_BACKEND
		MJR_ERROR("No rendering backend available. Cannot render offscreen!");
		can_render = false;
#endif

		if (!can_render) {
			settings_.render_offscreen = false;
			MJR_ERROR("Disabling offscreen rendering");
		} else {
			MJR_DEBUG("Starting offscreen render thread");
			offscreen_.render_thread_handle = std::thread(std::bind(&MujocoEnv::OffscreenRenderLoop, this));
		}
	}

	MJR_INFO_STREAM_COND(num_steps_until_exit_ > 0, "Sim will terminate after " << num_steps_until_exit_ << " steps");

	int available_threads = std::thread::hardware_concurrency() - 1;
	int num_threads       = 1;
	num_threads           = std::min(num_threads, available_threads);
	if (num_threads > 1) {
		threadpool_ = mju_threadPoolCreate(num_threads);
		MJR_INFO_STREAM("Using MuJoCo threadpool size of " << num_threads << " (max available: " << available_threads
		                                                   << ")");
	} else {
		MJR_INFO_STREAM("Running MuJoCo in single-threaded mode (" << available_threads << " threads available)");
	}

	// init VFS
	mj_defaultVFS(&vfs_);

	// setupServices();

	MujocoEnv::instance = this;

	mjcb_control = ProxyControlCB;
	mjcb_passive = ProxyPassiveCB;

	InitTFBroadcasting();
}

void MujocoEnv::RegisterCollisionFunction(int geom_type1, int geom_type2, mjfCollision collision_cb)
{
	if (custom_collisions_.find(std::pair(geom_type1, geom_type2)) != custom_collisions_.end() &&
	    custom_collisions_.find(std::pair(geom_type2, geom_type2)) != custom_collisions_.end()) {
		MJR_WARN_STREAM("A user defined collision callback for collisions between geoms of type "
		                << geom_type1 << " and " << geom_type2
		                << " have already been registered. This might lead to unexpected behavior!");
	} else {
		custom_collisions_.insert(std::pair(geom_type1, geom_type2));
		defaultCollisionFunctions.emplace_back(
		    CollisionFunctionDefault(geom_type1, geom_type2, mjCOLLISIONFUNC[geom_type1][geom_type2]));
	}
	mjCOLLISIONFUNC[geom_type1][geom_type2] = collision_cb;
}

void MujocoEnv::RegisterStaticTransform(TransformStamped &transform)
{
	MJR_DEBUG_STREAM("Registering static transform for frame " << transform.child_frame_id);
	for (auto it = static_transforms_.begin(); it != static_transforms_.end();) {
		if (it->child_frame_id == transform.child_frame_id) {
			MJR_WARN_STREAM("Static transform for child '" << transform.child_frame_id
			                                               << "' already registered. Will overwrite old transform!");
			static_transforms_.erase(it);
			break;
		}
		++it;
	}

	static_transforms_.emplace_back(transform);

	static_broadcaster_->sendTransform(static_transforms_);
}

void MujocoEnv::EventLoop()
{
	MJR_DEBUG("Starting event loop");
	is_event_running_ = 1;
	auto now          = Clock::now();
	auto fps_cap      = Seconds(mujoco_ros::Viewer::render_ui_rate_upper_bound_); // Cap at 60 fps
	while (roscpp::ok() && !settings_.exit_request.load()) {
		{
			RecursiveLock lock(physics_thread_mutex_);
			now = Clock::now();

			// if (settings_.settings_changed.load()) {
			// 	settings_.settings_changed.store(0);
			// 	updateDynamicParams();
			// }

			if (settings_.load_request.load() == 1) {
				MJR_DEBUG("Load request received");
				LoadWithModelAndData();
				MJR_DEBUG("Done loading");

				mnew = nullptr;
				dnew = nullptr;
				settings_.load_request.store(0);
				sim_state_.load_count += 1;
			} else if (settings_.load_request.load() >= 2) { // Loading mnew and dnew requested
				MJR_DEBUG("Initializing queued model and data");
				if (InitModelFromQueue()) {
					MJR_DEBUG("Init for load done. Requesting next load step");
					settings_.load_request.store(1);
				} else {
					MJR_ERROR("Init for load failed. Aborting load request");
					mj_deleteData(dnew);
					mj_deleteModel(mnew);
					mnew = nullptr;
					dnew = nullptr;
					settings_.load_request.store(0);
					sim_state_.load_count += 1;
				}
			}

			if (settings_.reset_request.load()) {
				ResetSim();
			}
		}

		std::this_thread::sleep_for(fps_cap - now.time_since_epoch());
	}
	MJR_DEBUG("Closing all connected viewers");
	for (const auto viewer : connected_viewers_) {
		viewer->exit_request.store(1);
	}
	MJR_DEBUG("Exiting event loop");
	is_event_running_ = 0;
}

void MujocoEnv::ResetSim()
{
	MJR_DEBUG("Sleeping to ensure all (old) ROS messages are sent");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	MJR_DEBUG("Resetting simulation environment");

	mj_resetData(this->model_.get(), this->data_.get());
	LoadInitialJointStates();
	ros_api_->PublishSimTime(this->data_->time);

	for (auto &plugin : plugins_) {
		plugin->SafeReset();
		MJR_DEBUG_STREAM("Resetting plugin " << plugin->get_type() << " took " << plugin->get_reset_time() << "seconds");
	}

	for (const auto viewer : connected_viewers_) {
		viewer->reset_request.store(1);
	}
	settings_.reset_request.store(0);
}

void MujocoEnv::LoadInitialJointStates()
{
	std::map<std::string, std::vector<double>> pos_map;
	std::map<std::string, std::vector<double>> vel_map;

	GetInitialJointPositions(pos_map);
	GetInitialJointVelocities(vel_map);

	// Change joint positions and velocities in between physics steps
	RecursiveLock lock(physics_thread_mutex_);

	// Joint positions
	for (auto const &[name, axis_vals] : pos_map) {
		MJR_DEBUG_STREAM("Trying to set jointpos of joint " << name);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			MJR_WARN_STREAM("Joint with name '" << name << "' could not be found. Initial joint position cannot be set!");
			continue;
		}

		int num_axes = 0;
		int jnt_type = model_->jnt_type[id];
		switch (jnt_type) {
			case mjJNT_FREE:
				num_axes = 7; // x y z (Position) w x y z (Orientation Quaternion) in world
				break;
			case mjJNT_BALL:
				num_axes = 4; // w x y z (Quaternion)
				break;
			case mjJNT_SLIDE:
			case mjJNT_HINGE:
				num_axes = 1; // single axis value
				break;
			default:
				continue;
		}

		if (axis_vals.size() != num_axes) {
			MJR_ERROR_STREAM("Provided initial position values for joint "
			                 << name << " don't match the degrees of freedom of the joint (exactly " << num_axes
			                 << " values are needed)!");
			continue;
		}
		for (int jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			SetJointPosition(axis_vals[jnt_axis], id, jnt_axis);
		}
	}

	// Joint velocities
	for (auto const &[name, axis_vals] : vel_map) {
		MJR_DEBUG_STREAM("Trying to set jointvels of joint " << name);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			MJR_WARN_STREAM("Joint with name '" << name << "' could not be found. Initial joint velocity cannot be set!");
			continue;
		}

		int num_axes = 0;
		int jnt_type = model_->jnt_type[id];
		switch (jnt_type) {
			case mjJNT_FREE:
				num_axes = 6; // x y z r p y
				break;
			case mjJNT_BALL:
				num_axes = 3; // r p y
				break;
			case mjJNT_SLIDE:
			case mjJNT_HINGE:
				num_axes = 1; // single axis value
				break;
			default:
				continue;
		}

		if (axis_vals.size() != num_axes) {
			MJR_ERROR_STREAM("Provided initial velocity values for joint "
			                 << name << " don't match the degrees of freedom of the joint (exactly " << num_axes
			                 << " values are needed)!");
			continue;
		}
		for (int jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			SetJointVelocity(axis_vals[jnt_axis], id, jnt_axis);
		}
	}
	// Apply changes in forward dynamics
	mj_forward(model_.get(), data_.get());
}

void MujocoEnv::ConnectViewer(Viewer *viewer)
{
	if (connected_viewers_.empty()) {
		MJR_INFO("Connected first viewer, disabling headless mode");
		settings_.headless = false;
	}
	while (GetOperationalStatus() != 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	MJR_DEBUG("Adding viewer to connected viewers and issuing viewer load request");
	if (std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer) == connected_viewers_.end()) {
		connected_viewers_.emplace_back(viewer);
		if (sim_state_.model_valid) {
			viewer->mnew_ = model_;
			viewer->dnew_ = data_;
			mju::strcmp_arr(viewer->filename, filename_);
			viewer->loadrequest = true;
		}
		return;
	}
	MJR_WARN("Viewer already connected!");
}

void MujocoEnv::DisconnectViewer(Viewer *viewer)
{
	auto it = std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer);
	if (it != connected_viewers_.end()) {
		MJR_DEBUG("Removing viewer from connected viewers");
		connected_viewers_.erase(it);
	} else {
		MJR_WARN("Viewer not connected!");
	}

	if (connected_viewers_.empty()) {
		MJR_INFO_COND(settings_.exit_request == 0, "Disconnected last viewer, enabling headless mode");
		settings_.headless = true;
	}
}

void MujocoEnv::NotifyGeomChanged(const int geom_id)
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->OnGeomChanged(this->model_.get(), this->data_.get(), geom_id);
	}
}

bool MujocoEnv::VerifyAdminHash(const std::string &hash)
{
	if (settings_.eval_mode) {
		MJR_DEBUG("Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != hash) {
			return false;
		}
		MJR_DEBUG("Hash valid, request authorized.");
	}
	return true;
}

void MujocoEnv::RunControlCbs()
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->WrappedControlCallback(this->model_.get(), this->data_.get());
	}
}

void MujocoEnv::RunPassiveCbs()
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->WrappedPassiveCallback(this->model_.get(), this->data_.get());
	}
}

MujocoEnv::~MujocoEnv()
{
	MJR_DEBUG("Destructor called");
	model_.reset();
	data_.reset();
	connected_viewers_.clear();
	free(this->ctrlnoise_);
	this->cb_ready_plugins_.clear();
	this->plugins_.clear();
	mj_deleteVFS(&vfs_);

	if (threadpool_ != nullptr) {
		mju_threadPoolDestroy(threadpool_);
	}

	plugin_utils::UnloadPluginloader();
}

} // namespace mujoco_ros
