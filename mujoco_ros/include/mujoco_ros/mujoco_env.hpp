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
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
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
 *   * Neither the name of Bielefeld University nor Neura Robotics nor the names of their
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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iterator>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

#if MJR_ROS_VERSION == ROS_1

#include <ros/ros.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/TransformStamped.h>

#include <mujoco_ros/ros_one/ros_api.hpp>

#include <dynamic_reconfigure/server.h>
#include <mujoco_ros/SimParamsConfig.h>

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
#include <mujoco_ros/mesh_uri_prep.hpp>
#include <mujoco_ros/simulation_control_state.hpp>
#include <mujoco_ros/runtime_options.hpp>
#include <mujoco_ros/plugin_host.hpp>
#include <mujoco_ros/rendering/render_core.hpp>
#include <mujoco_ros/rendering/render_snapshot.hpp>
#include <mujoco_ros/offscreen_transport.hpp>
#include <mujoco_ros/viewer.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/glfw_dispatch.h>
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

struct EnvSettings
{
	EnvSettings() = default;
	EnvSettings(const EnvSettings &other)
	    : headless(other.headless)
	    , render_offscreen(other.render_offscreen)
	    , use_sim_time(other.use_sim_time)
	    , render_backpressure_policy(other.render_backpressure_policy)
	    , busywait(other.busywait)
	    , num_mj_threads(other.num_mj_threads)
	    , eval_mode(other.eval_mode)
	    , visual_init_request(other.visual_init_request.load())
	    , settings_changed(other.settings_changed.load())
	    , is_python_request(other.is_python_request.load())
	{
		std::copy(std::begin(other.admin_hash), std::end(other.admin_hash), std::begin(admin_hash));
	}

	EnvSettings &operator=(const EnvSettings &other)
	{
		if (this == &other) {
			return *this;
		}
		headless                   = other.headless;
		render_offscreen           = other.render_offscreen;
		use_sim_time               = other.use_sim_time;
		render_backpressure_policy = other.render_backpressure_policy;
		busywait                   = other.busywait;
		num_mj_threads             = other.num_mj_threads;
		eval_mode                  = other.eval_mode;
		std::copy(std::begin(other.admin_hash), std::end(other.admin_hash), std::begin(admin_hash));
		visual_init_request.store(other.visual_init_request.load());
		settings_changed.store(other.settings_changed.load());
		is_python_request.store(other.is_python_request.load());
		return *this;
	}

	// Render options
	bool headless                                                  = false;
	bool render_offscreen                                          = false;
	bool use_sim_time                                              = true;
	rendering::RenderBackpressurePolicy render_backpressure_policy = rendering::RenderBackpressurePolicy::kDrop;

	// Sim speed configuration
	int busywait       = 0;
	int num_mj_threads = 1;

	// Mode
	bool eval_mode      = false;
	char admin_hash[64] = "\0";

	// Internal loading and configuration markers. Lifecycle control lives in SimulationControlState.
	std::atomic_int visual_init_request = { 0 };
	std::atomic_int settings_changed    = { 0 };

	// Must be set to true before loading a new model from python
	std::atomic_int is_python_request = { 0 };
};

struct SimState
{
	float measured_slowdown = 1.0;
	bool model_valid        = false;
	uint load_count         = 0;
};

struct SimInfo
{
	std::string model_path;
	bool model_valid = false;
	int load_count   = 0;

	int loading_state = 0;
	std::string loading_description;

	bool paused           = true;
	int pending_sim_steps = 0;
	float rt_measured     = 1.0f;
	float rt_setting      = 1.0f;
};

struct RosAPISettings
{
	bool running = false;
	std::string admin_hash;
	rendering::RenderBackpressurePolicy render_backpressure_policy = rendering::RenderBackpressurePolicy::kDrop;
};

#if MJR_ROS_VERSION == ROS_1
class MujocoEnv
{
public:
	/**
	 * @brief Construct a new Mujoco Env object.
	 *
	 */
	MujocoEnv(const std::string &admin_hash = std::string(), bool python_reload_service = false,
	          bool create_gui_adapter = true);
#else // MJR_ROS_VERSION == ROS_2
class MujocoEnv : public rclcpp::Node
{
public:
	/**
	 * @brief Construct a new Mujoco Env object.
	 *
	 */
	MujocoEnv(rclcpp::Executor::SharedPtr executor, const std::string &admin_hash = std::string(),
	          bool auto_configure = true, bool python_reload_service = false, bool create_gui_adapter = true);

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

	bool UsesPythonReloadService() const { return python_reload_service_; }
	// Friend declaration of RosAPI for access to private members
	friend class RosAPI;
	friend class Viewer;

public:
	virtual ~MujocoEnv();

	enum class ReloadPhase
	{
		// kRenderQuiescenceStarted is emitted after submission admission closes while
		// physics_thread_mutex_ is held; observers must return without waiting for
		// physics or graphics. An exception aborts reload and enters cleanup.
		kRenderQuiescenceStarted,
		// Emitted after RenderCore and all accepted caller-side render turns are idle,
		// with no physics lock held. Observers must not wait for physics or graphics;
		// an exception aborts reload and enters cleanup.
		kRenderTurnsIdle,
		// Emitted before render resources are initialized/reconfigured, while reload admission remains closed.
		// No physics lock is held; observers must not wait for physics or graphics.
		// An exception aborts reload and enters cleanup.
		kRenderReconfigureStarted,
		// The old camera/frame association has been retired; emitted under physics_thread_mutex_.
		// Observers must return without waiting for physics or graphics. An exception
		// aborts reload and enters cleanup.
		kOldGenerationQuiesced,
		// Model/data ownership has changed; emitted under physics_thread_mutex_.
		// Observers must return without waiting for physics or graphics. An exception
		// aborts reload and enters cleanup.
		kModelSwapped,
		// The new model/data has completed its forward pass; emitted under physics_thread_mutex_.
		// Observers must return without waiting for physics or graphics. An exception
		// aborts reload and enters cleanup.
		kForwarded,
		// New render resources are coherent and connected viewers are loaded; the
		// final operational publication and this callback are emitted together under
		// physics_thread_mutex_ after render admission reopens. Observers must return
		// without waiting for physics or graphics. An exception aborts reload and
		// enters cleanup.
		kNewGenerationLoaded,
		// Emitted after reload failure cleanup has restored explicit no-model lifecycle state, outside
		// physics_thread_mutex_; an observer exception is captured and logged.
		kReloadFailed,
	};

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

	mutable MujocoEnvMutex physics_thread_mutex_;

	void ConnectViewer(Viewer *viewer);
	void DisconnectViewer(Viewer *viewer);

	char queued_filename_[kMaxFilenameLength] = "\0";

	EnvSettings settings_;
	SimState sim_state_;

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
	void Configure();
	rendering::FrameStatus GetRenderStatus() const;
	rendering::FrameStatus SetRenderBackpressurePolicy(rendering::RenderBackpressurePolicy policy);
	rendering::FrameStatus SetRenderBackpressurePolicy(const std::string &policy);
	rendering::RenderBackpressurePolicy GetRenderBackpressurePolicy() const;
	std::shared_ptr<rendering::RenderCore> ActiveRenderCore() const { return render_core_; }
	FrameGeneration PublishedFrameGeneration() const { return frame_generation_; }
	FrameGeneration ActiveFrameGeneration() const;
	RuntimeOptionsTransactionResult GetRuntimeOptions();
	RuntimeOptionsTransactionResult ApplyRuntimeOptions(const std::vector<RuntimeOptionInput> &input);
	void SetPendingRuntimeOptions(const std::vector<RuntimeOptionInput> &input);

	template <typename Func>
	decltype(auto) WithPluginAccess(Func &&func) const
	{
		RecursiveLock physics_lock(physics_thread_mutex_);
		if (!plugin_host_) {
			throw std::runtime_error("Plugin Host is not initialized");
		}
		auto access = plugin_host_->AcquireScopedAccess();
		return std::invoke(std::forward<Func>(func), access);
	}

	template <typename Func>
	decltype(auto) WithPluginAccess(PluginGeneration expected_generation, Func &&func) const
	{
		RecursiveLock physics_lock(physics_thread_mutex_);
		if (!plugin_host_) {
			throw std::runtime_error("Plugin Host is not initialized");
		}
		auto access = plugin_host_->AcquireScopedAccess(expected_generation);
		return std::invoke(std::forward<Func>(func), access);
	}

	template <typename Func>
	decltype(auto) WithPluginAccess(ModelGeneration expected_model_generation, PluginGeneration expected_generation,
	                                Func &&func) const
	{
		RecursiveLock physics_lock(physics_thread_mutex_);
		if (!plugin_host_) {
			throw std::runtime_error("Plugin Host is not initialized");
		}
		auto access = plugin_host_->AcquireScopedAccess(expected_model_generation, expected_generation);
		return std::invoke(std::forward<Func>(func), access);
	}

	std::vector<PluginHandle> GetPluginHandles() const;

	// Builds a MujocoEnv directly from URDF/SRDF (Extended Params come from SRDF
	// <extended_params>, not a separate parameter), bypassing the
	// ROS-param-driven FetchRosConfiguration path entirely -- useful for tests
	// and non-ROS embedding. Routes through the existing LoadModelFromString
	// .mjb-file path; adds no new protected load method. Throws std::runtime_error
	// if conversion or compilation fails.
	static std::unique_ptr<MujocoEnv> from_description(const std::string &urdf_path, const std::string &srdf_path,
	                                                   const MeshPrepOptions &mesh_options = {},
	                                                   bool generate_actuators             = false,
	                                                   const std::string &attach_prefix    = "");
	bool SetBodyState(const std::string &body_name, mjtNum *pose, mjtNum *twist, mjtNum &mass, bool set_pose,
	                  bool set_twist, bool set_mass, bool reset_qpos, const std::string &admin_hash = std::string(),
	                  char *status_message = nullptr, const int status_sz = 0);
	bool SetBodyInertialProperties(const std::string &body_name, mjtNum mass, const mjtNum *ipos,
	                               const mjtNum *principal_inertia, const mjtNum *iquat,
	                               const std::string &admin_hash = std::string(), char *status_message = nullptr,
	                               int status_sz = 0);
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
	EnvSettings GetSettings() const;
	SimState GetSimState() const;
	SimInfo GetSimInfo();
	bool SetRealTimeFactor(const float &rt_factor, const std::string &admin_hash = std::string(),
	                       char *status_message = nullptr, const int status_sz = 0);
	std::vector<PluginStat> GetPluginStats() const;
	int GetPluginStats(std::vector<std::string> &plugin_names, std::vector<std::string> &types,
	                   std::vector<double> &load_times, std::vector<double> &reset_times,
	                   std::vector<double> &ema_steptimes_control, std::vector<double> &ema_steptimes_passive,
	                   std::vector<double> &ema_steptimes_render, std::vector<double> &ema_steptimes_last_stage);

	/**
	 * @brief Get information about the current simulation state.
	 *
	 * In addition to the control-state load request, this function considers visual initialization to be part of
	 * the loading process.
	 *
	 * @return 0 if done loading, 1 if loading is in progress, 2 if loading has been requested.
	 */
	int GetOperationalStatus();
	bool WaitForOperationalStatusIdle(std::chrono::milliseconds timeout);

	static constexpr float percentRealTime[] = {
		-1, // unbound
		2000, 1000, 800, 600,  500,  400, 200,  150,  100, 80,  66,   50,  40,  33,   25,   20,  16,   13,   10, 8,
		6.6f, 5.0f, 4,   3.3f, 2.5f, 2,   1.6f, 1.3f, 1,   .8f, .66f, .5f, .4f, .33f, .25f, .2f, .16f, .13f, .1f
	};

	static MujocoEnv *instance;
	static void ProxyControlCB(const mjModel *m, mjData *d)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->RunControlCbs(m, d);
	}
	static void ProxyPassiveCB(const mjModel *m, mjData *d)
	{
		if (MujocoEnv::instance != nullptr)
			MujocoEnv::instance->RunPassiveCbs(m, d);
	}

	// Proxies to MuJoCo callbacks
	void RunControlCbs(const mjModel *, mjData *);
	void RunPassiveCbs(const mjModel *, mjData *);

	bool TogglePaused(bool paused, const std::string &admin_hash = std::string());
	SimulationControlSnapshot GetControlSnapshot() const;
	struct ManualStepRequest
	{
		bool accepted         = false;
		ManualStepToken token = 0;
	};
	bool RequestManualSteps(int num_steps);
	ManualStepRequest RequestManualStepsWithToken(int num_steps);
	ManualStepSnapshot GetManualStepSnapshot(ManualStepToken token) const;
	ManualStepSnapshot WaitForManualStepUpdate(ManualStepToken token, int observed_pending_steps) const;
	bool CancelManualSteps(ManualStepToken token = 0);
	void AcknowledgeManualStep(ManualStepToken token);
	void RequestReset();

#if RENDER_BACKEND == GLFW_BACKEND
	GlfwAdapter *gui_adapter_ = nullptr;
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
	virtual void OnReloadPhase(ReloadPhase) {}

	void SetPhysicsPreLockTestHook(std::function<void()> hook)
	{
		std::lock_guard<std::mutex> lock(physics_test_hook_mutex_);
		physics_pre_lock_test_hook_ = std::move(hook);
	}
	bool IsResetInProgressForTest() const { return reset_in_progress_.load(std::memory_order_acquire); }

	void SetResetRejectedTestHook(std::function<void()> hook)
	{
		std::lock_guard<std::mutex> lock(physics_test_hook_mutex_);
		reset_rejected_test_hook_ = std::move(hook);
	}
	void SetRetirementTestHook(std::function<void()> hook)
	{
		std::lock_guard<std::mutex> lock(physics_test_hook_mutex_);
		retirement_test_hook_ = std::move(hook);
	}
	std::size_t RenderCallbackSceneCountForTest() const
	{
		return render_callback_scene_count_.load(std::memory_order_relaxed);
	}
	void ResetBatchSnapshotAssemblyCountForTest() { batch_snapshot_assembly_count_.store(0, std::memory_order_relaxed); }
	std::size_t BatchSnapshotAssemblyCountForTest() const
	{
		return batch_snapshot_assembly_count_.load(std::memory_order_relaxed);
	}

	std::unique_ptr<IPluginAdapterFactory> plugin_factory_;
	std::unique_ptr<PluginHost> plugin_host_;
	std::shared_ptr<void> plugin_lifetime_ = std::make_shared<int>(0);
#if MJR_ROS_VERSION == ROS_1
	XmlRpc::XmlRpcValue rpc_plugin_config_;
#endif

	// Get warning message for diverged simulation if autoreset is enabled.
	const char *Diverged(int disableflags, const mjData *d);

	/**
	 * @brief Utility function that runs a step, publishes sim time, triggers last stage callbacks, and notifies
	 * offscreen rendering condition if enabled.
	 */
	void WrappedStep();

	// This variable keeps track of remaining steps if the environment was configured to terminate after a fixed number
	// of steps (-1 means no limit).
	int num_steps_until_exit_ = -1;

	// VFS for loading models from strings
	mjVFS vfs_{};

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
	bool python_reload_service_ = false;
	bool create_gui_adapter_    = true;

	void RunLastStageCbs();

	void NotifyGeomChanged(const int geom_id);
	bool VerifyAdminHash(const std::string &hash);
	mjtNum ResetSim();
	void ProcessReset();
	void ResetOffscreenCameras();
	class InFlightRenderTurn
	{
	public:
		InFlightRenderTurn(const InFlightRenderTurn &)            = delete;
		InFlightRenderTurn &operator=(const InFlightRenderTurn &) = delete;
		InFlightRenderTurn(InFlightRenderTurn &&other) noexcept : owner_(other.owner_) { other.owner_ = nullptr; }
		InFlightRenderTurn &operator=(InFlightRenderTurn &&other) noexcept
		{
			if (this != &other) {
				Release();
				owner_       = other.owner_;
				other.owner_ = nullptr;
			}
			return *this;
		}
		~InFlightRenderTurn();

	private:
		friend class MujocoEnv;
		explicit InFlightRenderTurn(MujocoEnv *owner) : owner_(owner) {}
		void Release();
		MujocoEnv *owner_;
	};
	std::optional<InFlightRenderTurn> BeginInFlightRenderTurn();
	void EndInFlightRenderTurn();
	void CloseRenderTurnAdmission();
	void OpenRenderTurnAdmission();
	void WaitForInFlightRenderTurns();
	bool RetireRenderResources(std::string *cleanup_errors = nullptr) noexcept;
	void HandleReloadFailure(const char *message) noexcept;

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
	std::atomic_bool reset_in_progress_   = { false };
	std::atomic_bool reload_in_progress_{ false };
	std::atomic<std::size_t> render_callback_scene_count_{ 0 };
	std::atomic<std::size_t> batch_snapshot_assembly_count_{ 0 };
	mutable std::mutex physics_test_hook_mutex_;
	std::function<void()> physics_pre_lock_test_hook_;
	std::function<void()> reset_rejected_test_hook_;
	std::function<void()> retirement_test_hook_;
	std::mutex render_turn_mutex_;
	std::condition_variable render_turn_idle_;
	bool render_turn_admission_open_         = true;
	std::size_t in_flight_render_turn_count_ = 0;

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
	void SetPaused(bool paused);
	void RequestReload();
	void RequestModelLoad(const std::string &filename);
	void RequestViewerReset();
	void RequestViewerShutdown();
	void SetRealTimeIndex(int real_time_index);
	void SetViewerRealTimeIndex(int real_time_index);
	bool HasManualStepRequest() const;
	void RecordCompletedManualStep();
	bool IsShutdownRequested() const;
	void RequestShutdown();
	void RequestLoad(int load_request);
	void ClearResetRequest();
	void MarkSpeedChanged();
	bool ConsumeSpeedChange();
	struct ControlSpeedSnapshot
	{
		int real_time_index = 0;
		bool speed_changed  = false;
	};
	ControlSpeedSnapshot ConsumeSpeedSettingsSnapshot();
	void ApplyPauseState(bool paused, bool notify_settings_changed = true);
	template <typename Func>
	void PublishLoadRequest(int load_request, Func &&publish_payload)
	{
		// EventLoop consumes both payload and request while holding this boundary. Keep the same
		// physics -> control lock order here so it cannot observe a partially published load.
		RecursiveLock physics_lock(physics_thread_mutex_);
		WithControlState([this, load_request, &publish_payload]() {
			publish_payload();
			control_state_.SetLoadRequest(load_request);
			if (load_request > 0) {
				startup_runtime_options_open_ = false;
				runtime_options_admission_epoch_.fetch_add(1);
			}
		});
	}

	void QueueModelFilenameForLoad(const std::string &filename, int load_request);
	void QueueModelAndDataForLoad(mjModel *model, mjData *data, const std::string &filename, bool python_owned);

	/**
	 * @brief Serialize control-state transitions and load-payload publication.
	 *
	 * SimulationControlState is the only lifecycle/control authority. EnvSettings contains configuration
	 * and internal loading markers only.
	 */
	template <typename Func>
	auto WithControlState(Func &&func)
	{
		std::lock_guard<std::mutex> lock(control_state_boundary_mutex_);
		if constexpr (std::is_void_v<std::invoke_result_t<Func &>>) {
			func();
		} else {
			return func();
		}
	}

	/**
	 * @brief Handles requests from other threads (viewers).
	 */
	void EventLoop();

	void CompleteEnvSetup();

	/**
	 * @brief Tries to load all configured plugins.
	 * This function is called when a new mjData object is assigned to the environment.
	 */
	void LoadPlugins(PluginGeneration generation);

	std::vector<rendering::OffscreenCameraPtr> InitializeRenderResources();
	struct RenderSnapshotBatch
	{
		std::vector<rendering::OffscreenCameraPtr> cameras;
		std::shared_ptr<const mjModel> model;
		std::shared_ptr<mjData> data;
		std::shared_ptr<const std::vector<mjvGeom>> plugin_geometry;
		std::vector<rendering::RenderPlan> plans;
		std::vector<std::uint64_t> accepted_publication_sequences;
		ModelGeneration model_generation;
		std::chrono::nanoseconds simulation_time{ 0 };
	};
	void SubmitRenderSnapshot(RenderSnapshotBatch batch);
	void ReconfigureRenderCore();

	CameraPublicationTransport camera_publication_transport_;
	rendering::SnapshotPool snapshot_pool_;
	std::shared_ptr<rendering::RenderCore> render_core_;
	rendering::ConsumerId render_consumer_;
	std::shared_ptr<const mjModel> render_model_copy_;
	mutable std::mutex render_policy_mutex_;
	std::mutex render_warning_mutex_;
	std::chrono::steady_clock::time_point last_frame_slot_warning_{};
	void WarnFrameSlotDrop(const rendering::FrameStatus &status);
	// Keep storage unconditional: downstream consumers may compile without
	// MJR_BUILD_TESTING, and public object layout must remain identical.
	std::atomic<std::size_t> frame_slot_warning_count_{ 0 };
	std::function<std::chrono::steady_clock::time_point()> warning_clock_for_testing_;
	std::string last_frame_slot_warning_message_;
#ifdef MJR_BUILD_TESTING
	std::size_t FrameSlotWarningCountForTesting() const { return frame_slot_warning_count_.load(); }
	void SetWarningClockForTesting(std::function<std::chrono::steady_clock::time_point()> clock)
	{
		warning_clock_for_testing_ = std::move(clock);
	}
	const std::string &LastFrameSlotWarningForTesting() const { return last_frame_slot_warning_message_; }
#endif

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

	std::mutex control_state_boundary_mutex_;
	SimulationControlState control_state_;
	RuntimeOptionsPatch pending_runtime_options_;
	bool has_pending_runtime_options_  = false;
	bool startup_runtime_options_open_ = true;
	std::atomic<std::uint64_t> runtime_options_admission_epoch_{ 0 };
	OptionsEpoch options_epoch_;
	ModelGeneration model_generation_;
	PluginGeneration plugin_generation_;
	FrameGeneration frame_generation_;
};

} // end namespace mujoco_ros

namespace mujoco_ros {

template <typename Func>
decltype(auto) PluginHandle::WithAccess(Func &&func) const
{
	if (env_ == nullptr || lifetime_.expired()) {
		throw std::runtime_error("plugin handle belongs to a destroyed MujocoEnv");
	}
	return env_->WithPluginAccess(model_generation_, generation_, std::forward<Func>(func));
}

template <typename Backend, typename Func>
decltype(auto) PluginHandle::WithBackend(Func &&func) const
{
	return WithAccess([this, &func](const ScopedPluginAccess &access) -> decltype(auto) {
		auto *backend = access.BackendObject<Backend>(name_, type_);
		return std::invoke(std::forward<Func>(func), *backend);
	});
}

} // namespace mujoco_ros
