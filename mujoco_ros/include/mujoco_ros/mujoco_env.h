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

#pragma once

#include <thread>
#include <ros/ros.h>

#include <boost/thread.hpp>

#include <mujoco_ros/render_backend.h>
#include <mujoco_ros/common_types.h>
#include <mujoco_ros/viewer.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>
#include <actionlib/server/simple_action_server.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/Reload.h>
#include <std_srvs/Empty.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/GetGeomProperties.h>
#include <mujoco_ros_msgs/EqualityConstraintParameters.h>
#include <mujoco_ros_msgs/GetEqualityConstraintParameters.h>
#include <mujoco_ros_msgs/SetEqualityConstraintParameters.h>
#include <mujoco_ros_msgs/SetGravity.h>
#include <mujoco_ros_msgs/GetGravity.h>
#include <mujoco_ros_msgs/GetStateUint.h>
#include <mujoco_ros_msgs/GetSimInfo.h>
#include <mujoco_ros_msgs/SetFloat.h>
#include <mujoco_ros_msgs/PluginStats.h>
#include <mujoco_ros_msgs/GetPluginStats.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <mujoco_ros/SimParamsConfig.h>

#include <rosgraph_msgs/Clock.h>

#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/glfw_dispatch.h>
#elif RENDER_BACKEND == OSMESA_BACKEND
#include <GL/osmesa.h>
#endif

namespace mujoco_ros {

class MujocoEnvMutex : public std::recursive_mutex
{};
using MutexLock = std::unique_lock<std::recursive_mutex>;

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

	boost::thread render_thread_handle;

	// Condition variable to signal that the offscreen render thread should render a new frame
	std::atomic_bool request_pending = { false };

	std::mutex render_mutex;
	std::condition_variable_any cond_render_request;

	std::vector<rendering::OffscreenCameraPtr> cams;

	~OffscreenRenderContext();
};

class MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object.
	 *
	 */
	MujocoEnv(const std::string &admin_hash = std::string());
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

	void connectViewer(Viewer *viewer);
	void disconnectViewer(Viewer *viewer);

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

	std::vector<MujocoPluginPtr> const &getPlugins() const { return plugins_; }

	/**
	 * @brief Register a custom collision function for collisions between two geom types.
	 *
	 * @param [in] geom_type1 first geom type of the colliding geoms.
	 * @param [in] geom_type2 second type of the colliding geoms.
	 * @param [in] collision_cb collision function to call.
	 */
	void registerCollisionFunction(int geom_type1, int geom_type2, mjfCollision collision_cb);

	/**
	 * @brief Register a static transform to be published by the simulation.
	 *
	 * @param [in] transform const pointer to transform that will be published.
	 */
	void registerStaticTransform(geometry_msgs::TransformStamped &transform);

	void waitForPhysicsJoin();
	void waitForEventsJoin();

	void startPhysicsLoop();
	void startEventLoop();

	/**
	 * @brief Get information about the current simulation state.
	 *
	 * Additionally to the `settings_.load_request` state, this function also considers visual initialization to be a
	 * part of the loading process.
	 *
	 * @return 0 if done loading, 1 if loading is in progress, 2 if loading has been requested.
	 */
	int getOperationalStatus();

	static constexpr float percentRealTime[] = {
		-1, // unbound
		2000, 1000, 800, 600,  500,  400, 200,  150,  100, 80,  66,   50,  40,  33,   25,   20,  16,   13,   10, 8,
		6.6f, 5.0f, 4,   3.3f, 2.5f, 2,   1.6f, 1.3f, 1,   .8f, .66f, .5f, .4f, .33f, .25f, .2f, .16f, .13f, .1f
	};

	static MujocoEnv *instance;
	static void proxyControlCB(const mjModel * /*m*/, mjData * /*d*/)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->runControlCbs();
	}
	static void proxyPassiveCB(const mjModel * /*m*/, mjData * /*d*/)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->runPassiveCbs();
	}

	// Proxies to MuJoCo callbacks
	void runControlCbs();
	void runPassiveCbs();

	bool togglePaused(bool paused, const std::string &admin_hash = std::string());

#if RENDER_BACKEND == GLFW_BACKEND
	GlfwAdapter *gui_adapter_ = nullptr;
#endif

#if RENDER_BACKEND == EGL_BACKEND || RENDER_BACKEND == OSMESA_BACKEND
	bool InitGL();
#endif

	void runRenderCbs(mjvScene *scene);
	bool step(int num_steps = 1, bool blocking = true);

	void UpdateModelFlags(const mjOption *opt);

protected:
	std::vector<MujocoPlugin *> cb_ready_plugins_; // objects managed by plugins_
	XmlRpc::XmlRpcValue rpc_plugin_config_;
	std::vector<MujocoPluginPtr> plugins_;

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
	std::vector<geometry_msgs::TransformStamped> static_transforms_;

	// Central broadcaster for all static transforms
	tf2_ros::StaticTransformBroadcaster static_broadcaster_;

	// ROS TF2
	std::unique_ptr<tf2_ros::Buffer> tf_bufferPtr_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listenerPtr_;

	/// Pointer to mjModel
	mjModelPtr model_; // technically could be a unique_ptr, but setting the deleter correctly is not trivial
	/// Pointer to mjData
	mjDataPtr data_; // technically could be a unique_ptr, but setting the deleter correctly is not trivial

	std::vector<Viewer *> connected_viewers_;

	void publishSimTime(mjtNum time);
	ros::Publisher clock_pub_;
	std::unique_ptr<ros::NodeHandle> nh_;

	void runLastStageCbs();

	void notifyGeomChanged(const int geom_id);

	boost::recursive_mutex sim_params_mutex_;
	dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig> *param_server_;
	mujoco_ros::SimParamsConfig sim_params_;
	void dynparamCallback(mujoco_ros::SimParamsConfig &config, uint32_t level);
	void updateDynamicParams();

	std::vector<ros::ServiceServer> service_servers_;
	std::unique_ptr<actionlib::SimpleActionServer<mujoco_ros_msgs::StepAction>> action_step_;

	bool verifyAdminHash(const std::string &hash);

	void setupServices();
	bool setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &res);
	bool shutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool reloadCB(mujoco_ros_msgs::Reload::Request &req, mujoco_ros_msgs::Reload::Response &res);
	bool resetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool setBodyStateCB(mujoco_ros_msgs::SetBodyState::Request &req, mujoco_ros_msgs::SetBodyState::Response &resp);
	bool getBodyStateCB(mujoco_ros_msgs::GetBodyState::Request &req, mujoco_ros_msgs::GetBodyState::Response &resp);
	bool setGravityCB(mujoco_ros_msgs::SetGravity::Request &req, mujoco_ros_msgs::SetGravity::Response &resp);
	bool getGravityCB(mujoco_ros_msgs::GetGravity::Request &req, mujoco_ros_msgs::GetGravity::Response &resp);
	bool setGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
	                         mujoco_ros_msgs::SetGeomProperties::Response &resp);
	bool getGeomPropertiesCB(mujoco_ros_msgs::GetGeomProperties::Request &req,
	                         mujoco_ros_msgs::GetGeomProperties::Response &resp);
	bool setEqualityConstraintParametersArrayCB(mujoco_ros_msgs::SetEqualityConstraintParameters::Request &req,
	                                            mujoco_ros_msgs::SetEqualityConstraintParameters::Response &resp);
	bool setEqualityConstraintParameters(const mujoco_ros_msgs::EqualityConstraintParameters &parameters);
	bool getEqualityConstraintParametersArrayCB(mujoco_ros_msgs::GetEqualityConstraintParameters::Request &req,
	                                            mujoco_ros_msgs::GetEqualityConstraintParameters::Response &resp);
	bool getEqualityConstraintParameters(mujoco_ros_msgs::EqualityConstraintParameters &parameters);

	bool getStateUintCB(mujoco_ros_msgs::GetStateUint::Request &req, mujoco_ros_msgs::GetStateUint::Response &resp);
	bool getSimInfoCB(mujoco_ros_msgs::GetSimInfo::Request &req, mujoco_ros_msgs::GetSimInfo::Response &resp);
	bool setRTFactorCB(mujoco_ros_msgs::SetFloat::Request &req, mujoco_ros_msgs::SetFloat::Response &resp);
	bool getPluginStatsCB(mujoco_ros_msgs::GetPluginStats::Request &req,
	                      mujoco_ros_msgs::GetPluginStats::Response &resp);

	// Action calls
	void onStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal);

	void resetSim();

	/**
	 * @brief Loads and sets the initial joint states from the parameter server.
	 */
	void loadInitialJointStates();

	void setJointPosition(const double &pos, const int &joint_id, const int &jnt_axis /*= 0*/);
	void setJointVelocity(const double &vel, const int &joint_id, const int &jnt_axis /*= 0*/);

	/**
	 * @brief Makes sure that all data that will be replaced in a reload is freed.
	 */
	void prepareReload();

	// Threading

	boost::thread physics_thread_handle_;
	boost::thread event_thread_handle_;

	// Helper variables to get the state of threads
	std::atomic_int is_physics_running_   = { 0 };
	std::atomic_int is_event_running_     = { 0 };
	std::atomic_int is_rendering_running_ = { 0 };

	/**
	 * @brief Runs physics steps.
	 */
	void physicsLoop();

	/**
	 * @brief physics step when sim is running.
	 */
	void simUnpausedPhysics(mjtNum &syncSim, std::chrono::time_point<Clock> &syncCPU);

	/**
	 * @brief physics step when sim is paused.
	 */
	void simPausedPhysics(mjtNum &syncSim);

	/**
	 * @brief Handles requests from other threads (viewers).
	 */
	void eventLoop();

	void completeEnvSetup();

	/**
	 * @brief Tries to load all configured plugins.
	 * This function is called when a new mjData object is assigned to the environment.
	 */
	void loadPlugins();

	void initializeRenderResources();

	OffscreenRenderContext offscreen_;

	void offscreenRenderLoop();

	// Model loading
	mjModel *mnew = nullptr;
	mjData *dnew  = nullptr;

	/**
	 * @brief Load a queued model from either a path or XML-string.
	 */
	bool initModelFromQueue();

	/**
	 * @brief Replace the current model and data with new ones and complete the loading process.
	 */
	void loadWithModelAndData();

	mjThreadPool *threadpool_ = nullptr;
};

} // end namespace mujoco_ros
