// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#pragma once

#include <mujoco_ros/ros_version.hpp>

#include <atomic>
#include <thread>
#include <condition_variable>

#if MJR_ROS_VERSION == ROS_1

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TransformStamped.h>

#include <mujoco_ros/ros_one/ros_api.hpp>

using TransformStamped = geometry_msgs::TransformStamped;

#else // MJR_ROS_VERSION == ROS_2

#include <rclcpp/rclcpp.hpp>

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <mujoco_ros/ros_two/ros_api.hpp>

using TransformStamped = geometry_msgs::msg::TransformStamped;

#endif

#include <mujoco/mujoco.h>

#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/viewer.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// #include <dynamic_reconfigure/server.h>
// #include <mujoco_ros/SimParamsConfig.h>

#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/glfw_dispatch.h>
#elif RENDER_BACKEND == OSMESA_BACKEND
#include <GL/osmesa.h>
#endif

namespace mujoco_ros {

class MujocoEnvMutex : public std::recursive_mutex
{};
using RecursiveLock = std::unique_lock<std::recursive_mutex>;
using MutexLock     = std::unique_lock<std::mutex>;

struct CollisionFunctionDefault
{
	CollisionFunctionDefault(int geom_type1, int geom_type2, mjfCollision collision_cb)
	    : geom_type1_(geom_type1), geom_type2_(geom_type2), collision_cb_(collision_cb)
	{
	}

	int geom_type1_;
	int geom_type2_;
	mjfCollision collision_cb_;
};

struct OffscreenRenderContext
{
	mjvCamera cam;
	std::unique_ptr<unsigned char[]> rgb;
	std::unique_ptr<float[]> depth;
#if RENDER_BACKEND == GLFW_BACKEND
	std::shared_ptr<GLFWwindow> window;
#elif RENDER_BACKEND == OSMESA_BACKEND
	struct
	{
		OSMesaContext ctx;
		unsigned char buffer[10000000]; // TODO: size necessary or resize later?
		bool initialized = false;
	} osmesa;
#endif
	mjrContext con = {};
	mjvScene scn   = {};

	std::thread render_thread_handle;

	// Condition variable to signal that the offscreen render thread should render a new frame
	std::atomic_bool request_pending = { false };

	std::mutex render_mutex;
	std::condition_variable_any cond_render_request;

	std::vector<rendering::OffscreenCameraPtr> cams;

	~OffscreenRenderContext();
};

#if MJR_ROS_VERSION == ROS_1
class MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object.
	 *
	 */
	MujocoEnv(const std::string &admin_hash = std::string());
#else // MJR_ROS_VERSION == ROS_2
class MujocoEnv : public rclcpp::Node
{
public:
	/**
	 * @brief Construct a new Mujoco Env object.
	 *
	 */
	MujocoEnv(rclcpp::Executor::SharedPtr executor, const std::string &admin_hash = std::string());

	/**
	 * @brief Add a node to the executor of this server instance.
	 * This is a utility function to prevent subnodes needing to create their own executors
	 * inefficiently running separate threads. If the subnode is a direct plugin this will
	 * get called automatically, this is only be meant to be called manually in case plugins
	 * instanciate subnodes of their own.
	 */
	void AddNodeToExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

	/**
	 * @brief Remove a node to the executor of this server instance.
	 * This function gets called automatically on destructing a plugin.
	 * In case a plugin creates more subnodes, this function should be used on their destruction.
	 */
	void RemoveNodeFromExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node);

	/**
	 * @brief returns the executor_ shared pointer. 
	 * Some plugins (like ros2 control's controller manager) need direct reference to the executor.
	 */
	rclcpp::Executor::SharedPtr GetExecutorPtr();
#endif
	// Friend declaration of RosAPI for access to private members
	friend class RosAPI;

public:
	~MujocoEnv();

	MujocoEnv(const MujocoEnv &) = delete;

	// constants
	static constexpr int kErrorLength       = 1024;
	static constexpr int kMaxFilenameLength = 1000;

	const double syncMisalign       = 0.1; // maximum mis-alignment before re-sync (simulation seconds)
	const double simRefreshFraction = 0.7; // fraction of refresh available for simulation

	/// Noise to apply to control signal
	mjtNum *ctrlnoise_     = nullptr;
	double ctrl_noise_std  = 0.0;
	double ctrl_noise_rate = 0.0;

	mjvScene scn_;
	mjvPerturb pert_;

	MujocoEnvMutex physics_thread_mutex_;

	void ConnectViewer(Viewer *viewer);
	void DisconnectViewer(Viewer *viewer);

	char queued_filename_[kMaxFilenameLength] = "\0";

	struct
	{
		// Render options
		bool headless         = false;
		bool render_offscreen = false;
		bool use_sim_time     = true;

		// Sim speed
		int real_time_index = 8;
		int busywait        = 0;

		// Mode
		bool eval_mode      = false;
		char admin_hash[64] = "\0";

		// Atomics for multithread access
		std::atomic_int run                 = { 0 };
		std::atomic_int exit_request        = { 0 };
		std::atomic_int visual_init_request = { 0 };

		// Load request
		//  0: no request
		//  1: replace model_ with mnew and data_ with dnew
		//  2: load mnew and dnew from file
		std::atomic_int load_request      = { 0 };
		std::atomic_int reset_request     = { 0 };
		std::atomic_int speed_changed     = { 0 };
		std::atomic_int env_steps_request = { 0 };

		std::atomic_int settings_changed = { 0 };

		// Must be set to true before loading a new model from python
		std::atomic_int is_python_request = { 0 };
	} settings_;

	// General sim information for viewers to fetch
	struct
	{
		float measured_slowdown = 1.0;
		bool model_valid        = false;
		uint load_count         = 0;
	} sim_state_;

	std::vector<MujocoPluginPtr> const &GetPlugins() const
	{
		return plugins_;
	}

	/**
	 * @brief Register a custom collision function for collisions between two geom types.
	 *
	 * @param [in] geom_type1 first geom type of the colliding geoms.
	 * @param [in] geom_type2 second type of the colliding geoms.
	 * @param [in] collision_cb collision function to call.
	 */
	void RegisterCollisionFunction(int geom_type1, int geom_type2, mjfCollision collision_cb);

	/**
	 * @brief Register a static transform to be published by the simulation.
	 *
	 * @param [in] transform const pointer to transform that will be published.
	 */
	void RegisterStaticTransform(TransformStamped &transform);

	/////// Public API
	void StartPhysicsLoop();
	void StartEventLoop();

	void WaitForPhysicsJoin();
	void WaitForEventsJoin();

	void Shutdown();
	void Reset();
	bool LoadModelFromString(const std::string &model_xml, char *load_error = nullptr, const int error_sz = 0);
	bool SetBodyState(const std::string &body_name, mjtNum *pose, mjtNum *twist, mjtNum &mass, bool set_pose,
	                  bool set_twist, bool set_mass, bool reset_qpos, const std::string &admin_hash = std::string(),
	                  char *status_message = nullptr, const int status_sz = 0);
	bool GetBodyState(std::string &body_name, mjtNum *pose, mjtNum *twist, mjtNum *mass,
	                  const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                  const int status_sz = 0);
	bool SetGravity(const mjtNum *gravity, const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                const int status_sz = 0);
	bool GetGravity(mjtNum *gravity, const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                const int status_sz = 0);
	bool SetGeomProperties(const std::string &geom_name, const mjtNum body_mass, const mjtNum friction_slide,
	                       const mjtNum friction_spin, const mjtNum friction_roll, const mjtNum size_x,
	                       const mjtNum size_y, const mjtNum size_z, const mjtNum type, bool set_mass, bool set_friction,
	                       bool set_type, bool set_size, const std::string &admin_hash = std::string(),
	                       char *status_message = nullptr, const int status_sz = 0);
	bool GetGeomProperties(const std::string &geom_name, mjtNum &body_mass, mjtNum &friction_slide,
	                       mjtNum &friction_spin, mjtNum &friction_roll, mjtNum &size_x, mjtNum &size_y, mjtNum &size_z,
	                       mjtNum &type, const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                       const int status_sz = 0);
	bool SetEqualityConstraintParameters(const std::string &eq_name, const int &type, const mjtNum *solver_params,
	                                     const bool &active, const std::string &element1, const std::string &element2,
	                                     const mjtNum &torquescale, const mjtNum *anchor, const mjtNum *relpose,
	                                     const mjtNum *polycoef, const std::string &admin_hash = std::string(),
	                                     char *status_message = nullptr, const int status_sz = 0);
	bool GetEqualityConstraintParameters(const std::string &eq_name, int &type, mjtNum *solver_params, bool &active,
	                                     std::string &element1, std::string &element2, mjtNum &torquescale,
	                                     mjtNum *anchor, mjtNum *relpose, mjtNum *polycoef,
	                                     const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                                     const int status_sz = 0);
	void GetSimulationStatus(int &status, std::string &description);
	void GetSimInfo(std::string &model_path, bool &model_valid, int &load_count, int &loading_state,
	                std::string &loading_description, bool &paused, int &pending_sim_steps, float &rt_measured,
	                float &rt_setting);
	bool SetRealTimeFactor(const float &rt_factor, const std::string &admin_hash = std::string(),
	                       char *status_message = nullptr, const int status_sz = 0);
	int GetPluginStats(std::vector<std::string> &plugin_names, std::vector<std::string> &types,
	                   std::vector<double> &load_times, std::vector<double> &reset_times,
	                   std::vector<double> &ema_steptimes_control, std::vector<double> &ema_steptimes_passive,
	                   std::vector<double> &ema_steptimes_render, std::vector<double> &ema_steptimes_last_stage);

	/**
	 * @brief Get information about the current simulation state.
	 *
	 * Additionally to the `settings_.load_request` state, this function also considers visual initialization to be a
	 * part of the loading process.
	 *
	 * @return 0 if done loading, 1 if loading is in progress, 2 if loading has been requested.
	 */
	int GetOperationalStatus();

	static constexpr float percentRealTime[] = {
		-1, // unbound
		2000, 1000, 800, 600,  500,  400, 200,  150,  100, 80,  66,   50,  40,  33,   25,   20,  16,   13,   10, 8,
		6.6f, 5.0f, 4,   3.3f, 2.5f, 2,   1.6f, 1.3f, 1,   .8f, .66f, .5f, .4f, .33f, .25f, .2f, .16f, .13f, .1f
	};

	static MujocoEnv *instance;
	static void ProxyControlCB(const mjModel * /*m*/, mjData * /*d*/)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->RunControlCbs();
	}
	static void ProxyPassiveCB(const mjModel * /*m*/, mjData * /*d*/)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->RunPassiveCbs();
	}

	// Proxies to MuJoCo callbacks
	void RunControlCbs();
	void RunPassiveCbs();

	bool TogglePaused(bool paused, const std::string &admin_hash = std::string());

#if RENDER_BACKEND == GLFW_BACKEND
	GlfwAdapter *gui_adapter_ = nullptr;
#endif

#if RENDER_BACKEND == EGL_BACKEND || RENDER_BACKEND == OSMESA_BACKEND
	bool InitGL();
#endif

	void RunRenderCbs(mjvScene *scene);
	bool Step(int num_steps = 1, bool blocking = true);

	void UpdateModelFlags(const mjOption *opt);

	void FetchRosConfiguration();
	void GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type, float &pub_frequency,
	                            bool &use_segid, int &width, int &height, std::string &base_topic,
	                            std::string &rgb_topic, std::string &depth_topic, std::string &segment_topic);
	void GetInitialJointPositions(std::map<std::string, std::vector<double>> &joint_pos_map);
	void GetInitialJointVelocities(std::map<std::string, std::vector<double>> &joint_vel_map);

protected:
	std::vector<MujocoPlugin *> cb_ready_plugins_; // objects managed by plugins_
	std::vector<MujocoPluginPtr> plugins_;
#if MJR_ROS_VERSION == ROS_1
	XmlRpc::XmlRpcValue rpc_plugin_config_;
#endif

	// This variable keeps track of remaining steps if the environment was configured to terminate after a fixed number
	// of steps (-1 means no limit).
	int num_steps_until_exit_ = -1;

	// VFS for loading models from strings
	mjVFS vfs_;

	// Currently loaded model
	char filename_[kMaxFilenameLength] = "\0";
	// last error message
	char load_error_[kErrorLength] = "\0";

	// Store default collision functions to restore on reload
	std::vector<CollisionFunctionDefault> defaultCollisionFunctions;

	// Keep track of overriden collisions to throw warnings
	std::set<std::pair<int, int>> custom_collisions_;

	// Keep track of static transforms to publish.
	std::vector<TransformStamped> static_transforms_;

	void InitTFBroadcasting();
	// Central broadcaster for all static transforms
	std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

	// ROS TF2
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

	/// Pointer to mjModel
	mjModelPtr model_; // technically could be a unique_ptr, but setting the deleter correctly is not trivial
	/// Pointer to mjData
	mjDataPtr data_; // technically could be a unique_ptr, but setting the deleter correctly is not trivial

	std::vector<Viewer *> connected_viewers_;

	void PublishSimTime(mjtNum time);
#if MJR_ROS_VERSION == ROS_1
	std::shared_ptr<ros::NodeHandle> nh_;
#else // MJR_ROS_VERSION == ROS_2
	rclcpp::Executor::SharedPtr executor_;
#endif
	// ros api implementation
	std::unique_ptr<RosAPI> ros_api_;

	void Configure();

	void RunLastStageCbs();

	void NotifyGeomChanged(const int geom_id);
	bool VerifyAdminHash(const std::string &hash);
	void ResetSim();

	/**
	 * @brief Loads and sets the initial joint states from the parameter server.
	 */
	void LoadInitialJointStates();

	void SetJointPosition(const double &pos, const int &joint_id, const int &jnt_axis /*= 0*/);
	void SetJointVelocity(const double &vel, const int &joint_id, const int &jnt_axis /*= 0*/);

	/**
	 * @brief Makes sure that all data that will be replaced in a reload is freed.
	 */
	void PrepareReload();

	// Threading

	std::thread physics_thread_handle_;
	std::thread event_thread_handle_;

	// Helper variables to get the state of threads
	std::atomic_int is_physics_running_   = { 0 };
	std::atomic_int is_event_running_     = { 0 };
	std::atomic_int is_rendering_running_ = { 0 };

	/**
	 * @brief Runs physics steps.
	 */
	void PhysicsLoop();

	/**
	 * @brief physics step when sim is running.
	 */
	void SimUnpausedPhysics(mjtNum &syncSim, std::chrono::time_point<Clock> &syncCPU);

	/**
	 * @brief physics step when sim is paused.
	 */
	void SimPausedPhysics(mjtNum &syncSim);

	/**
	 * @brief Handles requests from other threads (viewers).
	 */
	void EventLoop();

	void CompleteEnvSetup();

	/**
	 * @brief Tries to load all configured plugins.
	 * This function is called when a new mjData object is assigned to the environment.
	 */
	void LoadPlugins();

	void InitializeRenderResources();

	OffscreenRenderContext offscreen_;

	void OffscreenRenderLoop();

	// Model loading
	mjModel *mnew = nullptr;
	mjData *dnew  = nullptr;

	/**
	 * @brief Load a queued model from either a path or XML-string.
	 */
	bool InitModelFromQueue();

	/**
	 * @brief Replace the current model and data with new ones and complete the loading process.
	 */
	void LoadWithModelAndData();

	mjThreadPool *threadpool_ = nullptr;
};

} // end namespace mujoco_ros
