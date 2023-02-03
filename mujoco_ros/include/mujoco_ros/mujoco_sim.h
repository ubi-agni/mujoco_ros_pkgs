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

#pragma once

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <condition_variable>

#include <tf2_ros/transform_listener.h>

#include <mujoco/mjxmacro.h>

#include <mujoco_ros/common_types.h>
#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/array_safety.h>

#include <mujoco_ros_msgs/SetPause.h>
#include <mujoco_ros_msgs/StepAction.h>
#include <mujoco_ros_msgs/StepGoal.h>
#include <mujoco_ros_msgs/SetBodyState.h>
#include <mujoco_ros_msgs/GetBodyState.h>
#include <mujoco_ros_msgs/SetGeomProperties.h>
#include <mujoco_ros_msgs/GetGeomProperties.h>
#include <mujoco_ros_msgs/SetGravity.h>
#include <mujoco_ros_msgs/GetGravity.h>

#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Empty.h>

namespace mju = ::mujoco::sample_util;

namespace MujocoSim {

#define mjENABLED_ros(model, x) (model->opt.enableflags & (x))
#define mjDISABLED_ros(model, x) (model->opt.disableflags & (x))

void init(std::string modelfile, const std::string &admin_hash);

void requestExternalShutdown(void);

// Threading and synchronization
static std::mutex sim_mtx;
static std::mutex render_mtx;
static std::condition_variable step_signal_;
static std::mutex readyness_mtx;
static int ready_threads_;

static simMode sim_mode_;
// To keep track of the amount of parallel environments
static int num_simulations_;

int jointName2id(mjModel *m, const std::string &joint_name, const std::string &robot_namespace = std::string());

void setJointPosition(mjModelPtr model, mjDataPtr data, const double &pos, const int &joint_id,
                      const int &jnt_axis = 0);
void setJointVelocity(mjModelPtr model, mjDataPtr data, const double &vel, const int &joint_id,
                      const int &jnt_axis = 0);

// Keep track of overriden collisions to throw warnings
static std::set<std::pair<int, int>> custom_collisions_;

/**
 * @brief Register a custom collision function for collisions between two geom types.
 *
 * @param [in] geom_type1 first geom type of the colliding geoms.
 * @param [in] geom_type2 second type of the colliding geoms.
 * @param [in] collision_cb collision function to call.
 */
void registerCollisionFunc(int geom_type1, int geom_type2, mjfCollision collision_cb);

/**
 * @brief Reset the simulation.
 *
 */
void resetSim();

/**
 * @brief Step through all simulated environments in a synchronized manner.
 *
 */
void synchedMultiSimStep();

/**
 * @brief Setup default VFS and save the current mujoco model in memory.
 *
 * @param [in] filename either the filename on disk or the VFS filename to save the provided content to.
 * @param [in] content model as string. Can be used if the model was constructed in memory and does not exist as file on
 * disk.
 */
void setupVFS(const std::string &filename, const std::string &content = std::string());

namespace detail {

// Env selected for rendering (if enabled) and responsible for time publishing.
extern MujocoEnvPtr main_env_;
// List of currently used envs
static std::vector<MujocoEnvPtr> env_list_;

// Time benchmarking bool for multienv performance measuring
static bool benchmark_env_time_;

// MuJoCo virtual filesystem to store mjcf in-memory.
static mjVFS vfs_;

// filename strings
static char filename_[kBufSize]          = "";
static char previous_filename_[kBufSize] = "";

// ROS callback for sim time publisher on /clock
void publishSimTime(mjtNum time);
static ros::Publisher pub_clock_;
static bool use_sim_time_;
static boost::shared_ptr<ros::NodeHandle> nh_;

// ROS TF2
static boost::shared_ptr<tf2_ros::Buffer> tf_bufferPtr_;
static boost::shared_ptr<tf2_ros::TransformListener> tf_listenerPtr_;

// Services
static std::vector<ros::ServiceServer> service_servers_;
// Actions
static std::unique_ptr<actionlib::SimpleActionServer<mujoco_ros_msgs::StepAction>> action_step_;

// constants
const double syncMisalign_ = 0.1; // maximum time mis-alignment before re-sync

void loadModel(void);
void loadInitialJointStates(mjModelPtr model, mjDataPtr data);
void setupEnv(MujocoEnvPtr env);

// Threads
void simulate(void);
void eventloop(void);
void envStepLoop(MujocoEnvParallelPtr env);

// Plugin callback proxies
void controlCallback(const mjModel *model, mjData *data);
void passiveCallback(const mjModel *model, mjData *data);
void renderCallback(mjData *data);
void lastStageCallback(mjData *data);

// Service calls
void setupCallbacks();
bool shutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
bool setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &resp);
bool resetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
bool setBodyStateCB(mujoco_ros_msgs::SetBodyState::Request &req, mujoco_ros_msgs::SetBodyState::Response &resp);
bool getBodyStateCB(mujoco_ros_msgs::GetBodyState::Request &req, mujoco_ros_msgs::GetBodyState::Response &resp);
bool setGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
                         mujoco_ros_msgs::SetGeomProperties::Response &resp);
bool getGeomPropertiesCB(mujoco_ros_msgs::GetGeomProperties::Request &req,
                         mujoco_ros_msgs::GetGeomProperties::Response &resp);
bool setGravityCB(mujoco_ros_msgs::SetGravity::Request &req, mujoco_ros_msgs::SetGravity::Response &resp);
bool getGravityCB(mujoco_ros_msgs::GetGravity::Request &req, mujoco_ros_msgs::GetGravity::Response &resp);

// Action calls
void onStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal);

// Sim settings not contained in MuJoCo structures
typedef struct _settings
{
	// atomics for multithreaded access
	std::atomic_int exitrequest       = { 0 };
	std::atomic_int visualInitrequest = { 0 };
	std::atomic_int run               = { 0 };
	std::atomic_int loadrequest       = { 0 };
	std::atomic_int resetrequest      = { 0 };
	std::atomic_bool speed_changed    = { true };
	// multi env
	std::atomic_int manual_env_steps = { 0 };

	// Time statistics
	float measured_slow_down = 1.0;
	int rt_index             = 1;

	// option
	int spacing    = 0;
	int color      = 0;
	int font       = 0;
	int ui0        = 1;
	int ui1        = 1;
	int help       = 0;
	int info       = 0;
	int profiler   = 0;
	int sensor     = 0;
	int fullscreen = 0;
	int vsync      = 1;
	int busywait   = 0;

	// simulation
	bool headless         = false;
	bool render_offscreen = false;
	int key               = 0;
	double ctrlnoisestd   = 0.0;
	double ctrlnoiserate  = 0.0;
	// Flag to toggle a more restrictive evaluation mode
	// otherwise ground truth topics are available for e.g. model training
	bool eval_mode = false;
	char admin_hash[64];

	// watch
	char field[mjMAXUITEXT] = "qpos";
	int index               = 0;

	// physics: need sync
	int disable[mjNDISABLE];
	int enable[mjNENABLE];

	// rendering: need sync
	int camera = 0;
} Settings;

extern Settings settings_;

static constexpr float percentRealTime[] = { -1, // unbound
	                                          100, 80, 66,  50,  40, 33, 25,  20, 16, 13,  10,  8,  6.6, 5.0, 4, 3.3,
	                                          2.5, 2,  1.6, 1.3, 1,  .8, .66, .5, .4, .33, .25, .2, .16, .13, .1 };

} // end namespace detail
} // end namespace MujocoSim
