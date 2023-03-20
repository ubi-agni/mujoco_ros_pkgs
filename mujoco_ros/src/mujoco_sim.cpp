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

/* Authors: David P. Leins*/

#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros/rendering/utils.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <rosgraph_msgs/Clock.h>
#include <boost/filesystem.hpp>

#include <mujoco_ros_msgs/GetStateUint.h>

#include <iostream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
namespace MujocoSim {

using namespace detail;

namespace mjsr = ::MujocoSim::rendering;

// Define extern declared globals
Settings MujocoSim::detail::settings_;
MujocoEnvPtr MujocoSim::detail::main_env_;

int jointName2id(mjModel *m, const std::string &joint_name, const std::string &robot_namespace /* = std::string()*/)
{
	int result = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
	if (result == -1 and robot_namespace.size() > 0) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Trying to find joint without namespace ("
		                                     << joint_name.substr(robot_namespace.size()) << ")");
		result = mj_name2id(m, mjOBJ_JOINT, joint_name.substr(robot_namespace.size()).c_str());
	}
	return result;
}

void registerCollisionFunc(int geom_type1, int geom_type2, mjfCollision collision_cb)
{
	if (custom_collisions_.find(std::pair(geom_type1, geom_type2)) != custom_collisions_.end() &&
	    custom_collisions_.find(std::pair(geom_type2, geom_type2)) != custom_collisions_.end()) {
		ROS_WARN_STREAM_NAMED("mujoco", "A user defined collision callback for collisions between geoms of type "
		                                    << geom_type1 << " and " << geom_type2
		                                    << " have already been registered. This might lead to unexpected behavior!");
	} else {
		custom_collisions_.insert(std::pair(geom_type1, geom_type2));
	}
	mjCOLLISIONFUNC[geom_type1][geom_type2] = collision_cb;
}

void registerStaticTransform(geometry_msgs::TransformStamped transform)
{
	ROS_DEBUG_STREAM_NAMED("mujoco", "Registering static transform for frame " << transform.child_frame_id);
	for (std::vector<geometry_msgs::TransformStamped>::iterator it = static_transforms_.begin();
	     it != static_transforms_.end();) {
		if (it->child_frame_id == transform.child_frame_id) {
			ROS_WARN_STREAM_NAMED("mujoco", "Static transform for child '"
			                                    << transform.child_frame_id
			                                    << "' already registered. Will overwrite old transform!");
			static_transforms_.erase(it);
			break;
		}
		++it;
	}

	static_transforms_.push_back(transform);

	static_broadcaster_->sendTransform(static_transforms_);
}

void setupVFS(const std::string &filename, const std::string &content /* = std::string()*/)
{
	mj_defaultVFS(&vfs_);

	// File from disk
	int ret;
	if (content.empty()) {
		if (!boost::filesystem::is_regular_file(filename)) {
			ROS_ERROR_STREAM_NAMED("mujoco", filename << " is not a regular file! Cannot load config!");
			return;
		}

		boost::filesystem::path path(filename);
		path = boost::filesystem::absolute(path);
		ret  = mj_addFileVFS(&vfs_, path.parent_path().c_str(), path.filename().c_str());
		if (ret == 1) {
			ROS_ERROR_STREAM_NAMED("mujoco", "Could not save file "
			                                     << filename
			                                     << " to VFS, because it's full! This should not happen, check with dev!");
			return;
		} else if (ret == 2) {
			ROS_ERROR_STREAM_NAMED("mujoco", "Could not save file "
			                                     << filename << " to VFS, because the file is already present in VFS!");
			return;
		}
		ROS_DEBUG_STREAM_NAMED("mujoco",
		                       "Successfully saved file " << filename << " in mujoco VFS for accelerated loading");
	} else { // File in memory
		mj_makeEmptyFileVFS(&vfs_, filename.c_str(), content.size() + 1);
		int file_idx = mj_findFileVFS(&vfs_, filename.c_str());
		memcpy(vfs_.filedata[file_idx], content.c_str(), content.size() + 1);
		ROS_DEBUG_STREAM_NAMED("mujoco", "Successfully saved xml content as mujoco VFS file for accelerated loading");
	}
}

void init(std::string modelfile, std::string admin_hash /* = std::string()*/)
{
	// use_sim_time should be set in roslaunch before running any node.
	// Otherwise nodes might behave unintendedly. Hence, issue an error in this case.
	if (!ros::param::get("/use_sim_time", use_sim_time_)) {
		ROS_FATAL_NAMED("mujoco", "/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		                          "or false. Also Make sure it is set before starting any node, "
		                          "otherwise nodes might behave unexpectedly.");
		return;
	}

	ROS_DEBUG_COND_NAMED(!use_sim_time_, "mujoco",
	                     "use_sim_time set to false. Not publishing simulation time as ROS time!");

	nh_.reset(new ros::NodeHandle("~"));

	if (use_sim_time_) {
		pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 1);
		publishSimTime(mjtNum(0));
	}

	settings_.exitrequest.store(0);

	nh_->param<bool>("eval_mode", settings_.eval_mode, false);
	if (settings_.eval_mode) {
		ROS_WARN_NAMED("mujoco", "Evalutaion mode is active. Looking for admin hash...");
		if (admin_hash == "") {
			ROS_ERROR_NAMED("mujoco", "Evaluation mode requires a hash to verify critical operations are allowed. No hash "
			                          "was provided, aborting launch.");
			mju_error("Evaluation mode requires a hash to verify critical operations are allowed. No hash was provided, "
			          "aborting launch.");
		} else {
			std::strcpy(settings_.admin_hash, admin_hash.c_str());
		}
	} else {
		ROS_WARN_NAMED("mujoco", "Train mode is active");
	}

	bool vis, no_x;
	nh_->param<bool>("visualize", vis, true);
	nh_->param<bool>("render_offscreen", settings_.render_offscreen, true);
	nh_->param<bool>("no_x", no_x, false);

	if (no_x) {
		ROS_WARN_NAMED("mujoco", "No X is active. Disabling visualization and offscreen rendering ...");
		vis                        = false;
		settings_.render_offscreen = false;
	}

	ROS_INFO_COND_NAMED(!vis, "mujoco", "Running in headless mode");
	settings_.headless = !vis;
	ROS_WARN_COND_NAMED(!settings_.render_offscreen, "mujoco", "Rendering offscreen camera streams is disabled!");

	// Print version, check compatibility
	ROS_INFO("MuJoCo Pro library version %.2lf\n", 0.01 * mj_version());
	if (mjVERSION_HEADER != mj_version()) {
		ROS_ERROR_STREAM_NAMED("mujoco", "Headers and library have different versions (headers: "
		                                     << mjVERSION_HEADER << " vs. lib: " << mj_version() << ")");
		mju_error("Headers and library have different versions");
	}

	static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster());

	// Check which sim mode to use
	int num_simulations;
	nh_->param<int>("num_simulations", num_simulations, 1);
	if (num_simulations > 1) {
		ROS_FATAL_NAMED("mujoco", "PARALLEL mode is not available anymore, because of several issues. Parallel "
		                          "processing will return in a later release.");
		settings_.exitrequest.store(1);
	}

	nh_->param<bool>("benchmark_time", benchmark_env_time_, false);

	bool unpause;
	nh_->param<bool>("unpause", unpause, true);
	std::string main_env_namespace;
	nh_->param<std::string>("ns", main_env_namespace, "/");
	if (main_env_namespace == "")
		main_env_namespace = "/";
	main_env_.reset(new MujocoEnv(main_env_namespace));

	if (unpause) {
		settings_.run.store(1);
	} else {
		ROS_DEBUG_NAMED("mujoco", "Starting in paused state");
		settings_.run.store(0);
	}

	if (!no_x) {
		mjsr::initVisual();
	}

	if (!modelfile.empty()) {
		std::strcpy(filename_, modelfile.c_str());
		settings_.loadrequest.store(2);
	}

	setupCallbacks();

	std::thread simthread(simulate);

	tf_bufferPtr_.reset(new tf2_ros::Buffer());
	tf_bufferPtr_->setUsingDedicatedThread(true);
	tf_listenerPtr_.reset(new tf2_ros::TransformListener(*tf_bufferPtr_));

	eventloop();

	ROS_DEBUG_NAMED("mujoco", "Event loop terminated");
	settings_.exitrequest.store(1);
	simthread.join();

	environments::unregisterEnv(main_env_->data.get());

	env_list_.clear();
	static_transforms_.clear();

	ROS_DEBUG_NAMED("mujoco", "Sim thread terminated");

	main_env_.reset();
	mjsr::deinitVisual();
	plugin_utils::unloadPluginloader();

	mj_deleteVFS(&vfs_);

	if (use_sim_time_) {
		pub_clock_.shutdown();
	}

	for (auto ss : service_servers_) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Shutting down service " << ss.getService());
		ss.shutdown();
	}
	service_servers_.clear();
	action_step_->shutdown();
	action_step_.reset();

	ROS_DEBUG_NAMED("mujoco", "Cleanup done");
	tf_bufferPtr_.reset();
	tf_listenerPtr_.reset();
	static_broadcaster_.reset();
	nh_.reset();
}

void setJointPosition(mjModelPtr model, mjDataPtr data, const double &pos, const int &joint_id,
                      const int &jnt_axis /*= 0*/)
{
	data->qpos[model->jnt_qposadr[joint_id] + jnt_axis]        = pos;
	data->qvel[model->jnt_dofadr[joint_id] + jnt_axis]         = 0;
	data->qfrc_applied[model->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void setJointVelocity(mjModelPtr model, mjDataPtr data, const double &vel, const int &joint_id,
                      const int &jnt_axis /*= 0*/)
{
	data->qvel[model->jnt_dofadr[joint_id] + jnt_axis]         = vel;
	data->qfrc_applied[model->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void requestExternalShutdown(void)
{
	settings_.exitrequest.store(1);
}

void resetSim()
{
	if (main_env_->model) {
		// sim_mtx is already locked
		ROS_DEBUG_NAMED("mujoco", "Sleeping to ensure all (old) ROS messages are sent");
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		ROS_DEBUG_NAMED("mujoco", "Starting reset");

		mj_resetData(main_env_->model.get(), main_env_->data.get());
		loadInitialJointStates(main_env_->model, main_env_->data);
		mj_forward(main_env_->model.get(), main_env_->data.get());
		publishSimTime(main_env_->data->time);

		if (!settings_.headless) {
			mjsr::profilerUpdate(main_env_);
			mjsr::sensorUpdate(main_env_);
			mjsr::updateSettings(main_env_);
		}
	}
	settings_.resetrequest.store(0);
}

namespace detail {

void eventloop(void)
{
	ros::WallTime t_last(0);
	ros::WallTime now(0);
	ros::WallDuration fps_cap(mjsr::render_ui_rate_upper_bound_); // Cap to 60 FPS
	while (ros::ok() && ((!settings_.exitrequest.load() && (settings_.headless || !mjsr::isWindowClosing())))) {
		{
			std::lock_guard<std::mutex> lk(sim_mtx);
			now = ros::WallTime::now();

			if (settings_.loadrequest.load() == 1) {
				loadModel();
			} else if (settings_.loadrequest.load() > 1) {
				settings_.loadrequest.store(1);
			}

			if (settings_.resetrequest.load() == 1) {
				resetSim();
			}

			if (!settings_.headless && now - t_last >= fps_cap) {
				// Handle events (via callbacks)
				glfwPollEvents();

				// Prepare to render
				mjsr::prepareOnScreen(t_last - now);
				t_last = now;
			}
		} // unlocks sim_mtx

		// render while sim is running
		if (!settings_.headless) {
			mjsr::renderMain();
		}
	}
}

void publishSimTime(mjtNum time)
{
	if (!use_sim_time_) {
		return;
	}
	// This is the fastes option for intra-node time updates
	// however, together with non-blocking publish it breaks stuff
	// ros::Time::setNow(ros::Time(time));
	rosgraph_msgs::ClockPtr ros_time(new rosgraph_msgs::Clock);
	ros_time->clock.fromSec(time);
	pub_clock_.publish(ros_time);
	// Current workaround to simulate a blocking time update
	while (ros::Time::now() < ros::Time(time)) {
		std::this_thread::yield();
	}
}

void controlCallback(const mjModel * /*model*/, mjData *data)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runControlCbs();
	}
}

void passiveCallback(const mjModel * /*model*/, mjData *data)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runPassiveCbs();
	}
}

void lastStageCallback(mjData *data)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runLastStageCbs();
	}
}

void simulate(void)
{
	int num_steps;
	nh_->param<int>("num_steps", num_steps, -1);

	ROS_DEBUG_STREAM_COND_NAMED(num_steps > 0, "mujoco", "Simulation will terminate after " << num_steps << " steps");

	// cpu-sim syncronization point
	ros::WallTime syncCPU(0); // last sync time in WallTime
	double elapsedCPU(0);
	mjtNum syncSim    = 0; // last sync time in sim time
	mjtNum elapsedSim = 0;

	mjModelPtr model;
	mjDataPtr data;

	while (!settings_.exitrequest.load() && num_steps != 0) {
		model = main_env_->model;
		data  = main_env_->data;

		if (!settings_.run.load() && settings_.busywait) {
			std::this_thread::yield();
		} else {
			std::this_thread::sleep_for(std::chrono::nanoseconds(10));
		}

		if (model && data) {
			if (settings_.visualInitrequest.load()) {
				if (settings_.render_offscreen) {
					main_env_->initializeRenderResources();
				}
				settings_.visualInitrequest.store(0);
			}

			std::lock_guard<std::mutex> lk(sim_mtx);
			if (settings_.run.load()) {
				// Wall time at the start of this iteration
				ros::WallTime startCPU = ros::WallTime::now();

				// Elapsed Wall and simulation time since last sync
				elapsedSim = data->time - syncSim;
				elapsedCPU = (startCPU - syncCPU).toSec();

				// Inject noise
				if (settings_.ctrlnoisestd) {
					// Convert rate and scale to discrete time given current timestep
					mjtNum rate  = mju_exp(-model->opt.timestep / settings_.ctrlnoiserate);
					mjtNum scale = settings_.ctrlnoisestd * mju_sqrt(1 - rate * rate);

					for (int i = 0; i < model->nu; i++) {
						// Update noise
						main_env_->ctrlnoise[i] = rate * main_env_->ctrlnoise[i] + scale * mju_standardNormal(nullptr);
						// Apply noise
						main_env_->data->ctrl[i] = main_env_->ctrlnoise[i];
					}
				}

				double slow_down = 100 / percentRealTime[settings_.rt_index];

				mjtNum misaligned = mju_abs(elapsedCPU / slow_down - elapsedSim) > syncMisalign_;

				// Out-of-sync
				if (elapsedSim < 0 || elapsedCPU < 0 || syncCPU.toSec() == 0 || misaligned ||
				    settings_.speed_changed.load()) {
					// Re-sync
					syncCPU = startCPU;
					syncSim = data->time;
					settings_.speed_changed.store(false);

					// Clear old perturbations, apply new
					mju_zero(data->xfrc_applied, 6 * model->nbody);
					mjv_applyPerturbPose(model.get(), data.get(), &mjsr::pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(model.get(), data.get(), &mjsr::pert_);

					// Run single step, let next iteration deal with timing
					mj_step(model.get(), data.get());
					publishSimTime(data->time);
					lastStageCallback(data.get());
					mjsr::offScreenRenderEnv(main_env_);

					// Count steps until termination
					if (num_steps > 0) {
						num_steps--;
						ROS_INFO_COND_NAMED(num_steps == 0, "mujoco", "running last sim step before termination!");
					}
				} else { // in-sync: step until ahead of CPU
					bool measured  = false;
					mjtNum prevSim = data->time;

					// If real-time is bound, run until sim steps are in sync with CPU steps, otherwise run as fast as
					// possible
					while ((settings_.rt_index == 0 ||
					        (data->time - syncSim) * slow_down < (ros::WallTime::now() - syncCPU).toSec()) &&
					       (ros::WallTime::now() - startCPU).toSec() < mjsr::render_ui_rate_lower_bound_ &&
					       (num_steps == -1 || num_steps > 0) && !settings_.exitrequest.load()) {
						if (!measured && elapsedSim) {
							settings_.measured_slow_down = elapsedCPU / elapsedSim;
							measured                     = true;
						}

						// clear old perturbations, apply new
						mju_zero(data->xfrc_applied, 6 * model->nbody);
						mjv_applyPerturbPose(model.get(), data.get(), &mjsr::pert_, 0); // Move mocap bodies only
						mjv_applyPerturbForce(model.get(), data.get(), &mjsr::pert_);

						// Run mj_step
						mj_step(model.get(), data.get());
						publishSimTime(data->time);
						lastStageCallback(data.get());
						mjsr::offScreenRenderEnv(main_env_);

						// Count steps until termination
						if (num_steps > 0) {
							num_steps--;
							ROS_INFO_COND_NAMED(num_steps == 0, "mujoco", "running last sim step before termination!");
						}

						// break on reset
						if (data->time < prevSim) {
							break;
						}
					}
				}
			} else { // Paused
				if (settings_.manual_env_steps.load() > 0) { // Action call or arrow keys used for stepping
					mju_zero(data->xfrc_applied, 6 * model->nbody);
					mjv_applyPerturbPose(model.get(), data.get(), &mjsr::pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(model.get(), data.get(), &mjsr::pert_);

					// Run single step
					mj_step(model.get(), data.get());
					publishSimTime(data->time);
					lastStageCallback(data.get());
					mjsr::offScreenRenderEnv(main_env_);

					settings_.manual_env_steps.store(settings_.manual_env_steps.load() - 1);
				} else { // Wating in paused mode
					mjv_applyPerturbPose(model.get(), data.get(), &mjsr::pert_, 1); // Move mocap and dynamic bodies

					// Run mj_forward, to update rendering and joint sliders
					mj_forward(model.get(), data.get());
					mjsr::offScreenRenderEnv(main_env_);
				}
			}
		} // end if(model && data) unlocks sim_mtx

		// TODO(dleins): This instantly loads the default modelfile when an xml is provided via parameter server
		// This should be handled differently, maybe only by service call?
		// std::string modelfile;
		// nh_->getParam("modelfile", modelfile);
		// if (!modelfile.empty() && strcmp(filename_, modelfile.c_str())) {
		// 	ROS_DEBUG_STREAM_NAMED("mujoco", "Got new modelfile from param server! Requesting load ...");
		// 	std::strcpy(filename_, modelfile.c_str());
		// 	settings_.loadrequest.store(2);
		// }
	}
	// Requests eventloop shutdown in case we ran out of simulation steps to use
	settings_.exitrequest.store(1);
}

void loadModel(void)
{
	ROS_DEBUG_NAMED("mujoco", "Loading model...");
	// Make sure filename is given
	if (!filename_) {
		return;
	}

	// Load and compile
	char error[500] = "";
	mjModel *mnew   = 0;
	if (mju::strlen_arr(filename_) > 4 && !std::strncmp(filename_ + mju::strlen_arr(filename_) - 4, ".mjb",
	                                                    mju::sizeof_arr(filename_) - mju::strlen_arr(filename_) + 4)) {
		mnew = mj_loadModel(filename_, NULL);
		if (!mnew) {
			mju::strcpy_arr(error, "could not load binary model");
		}
	} else {
		mnew = mj_loadXML(filename_, &vfs_, error, 500);
	}
	if (!mnew) {
		std::printf("%s\n", error);
		return;
	}

	// Compiler warning: print and pause
	if (error[0]) {
		// mj_forward() will print the warning message
		ROS_WARN_NAMED("mujoco", "Model compiled, but simulation warning: \n %s\n\n", error);
		std::printf("Model compiled, but simulation warning: \n %s\n\n", error);

		// Only pause if not in headless mode
		if (!settings_.headless) {
			settings_.run.store(0);
		}
	}

	ROS_DEBUG_STREAM_NAMED("mujoco", "Setting up env '" << main_env_->name << "' ...");
	// Delete old model, assign new
	main_env_->model.reset(mnew, [](mjModel *m) { mj_deleteModel(m); });
	environments::assignData(mj_makeData(mnew), main_env_);
	setupEnv(main_env_);
	// Request that off-screen rendering resources are initialized in simulate thread
	settings_.visualInitrequest.store(1);

	// Align and scale view unless reloading the same file
	bool realign = mju::strcmp_arr(filename_, previous_filename_);
	if (realign) {
		mju::strcpy_arr(previous_filename_, filename_);
	}

	// set real time index
	int numclicks   = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
	float min_error = 1e6;
	float desired;
	nh_->param<float>("realtime", desired, main_env_->model->vis.global.realtime);

	if (desired == -1) {
		settings_.rt_index = 0;
	} else if (desired <= 0 or desired > 1) {
		ROS_WARN_NAMED("mujoco", "Desired realtime should be greater than 0 and not greater than 1 or set to -1 to run "
		                         "at the fastest time possible. Setting to 1");
		settings_.rt_index = 1;
	} else {
		desired = mju_log(100 * desired);
		for (int click = 0; click < numclicks; click++) {
			float error = mju_abs(mju_log(percentRealTime[click]) - desired);
			if (error < min_error) {
				min_error          = error;
				settings_.rt_index = click;
			}
		}
	}

	mjsr::onModelLoad(main_env_, realign);
	settings_.loadrequest.store(0);
}

void setupEnv(MujocoEnvPtr env)
{
	loadInitialJointStates(env->model, env->data);

	mj_forward(env->model.get(), env->data.get());

	ROS_DEBUG_NAMED("mujoco", "resetting noise ...");
	// Allocate ctrlnoise
	free(env->ctrlnoise);
	env->ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * env->model->nu);
	mju_zero(env->ctrlnoise, env->model->nu);

	env->reload();
}

void loadInitialJointStates(mjModelPtr model, mjDataPtr data)
{
	ROS_DEBUG_NAMED("mujoco", "Fetching and setting initial joint positions ...");

	// Joint positions
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_positions/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be recognized.
	if (nh_->hasParam("initial_joint_positions/joint_map") && joint_map.size() == 0) {
		ROS_WARN_NAMED("mujoco", "Initial joint positions not recognized by rosparam server. Check your config, "
		                         "especially values for single axis joints should explicitly provided as string!");
	}
	for (auto const &[name, str_values] : joint_map) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Trying to set jointpos of joint " << name << " to values: " << str_values);
		int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			ROS_WARN_STREAM_NAMED("mujoco", "Joint with name '"
			                                    << name << "' could not be found. Initial joint position cannot be set!");
			continue;
		}

		int num_axes = 0;
		int jnt_type = model->jnt_type[id];
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
				break;
		}

		double axis_vals[num_axes];
		std::stringstream stream_values(str_values);
		int jnt_axis = 0;
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			if (jnt_axis > num_axes)
				break;
			axis_vals[jnt_axis] = std::stod(value);
			jnt_axis++;
		}

		if (jnt_axis != num_axes) {
			ROS_ERROR_STREAM_NAMED("mujoco", "Provided initial position values for joint "
			                                     << name << " don't match the degrees of freedom of the joint (exactly "
			                                     << num_axes << " values are needed)!");
			continue;
		}
		for (jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			setJointPosition(model, data, axis_vals[jnt_axis], id, jnt_axis);
		}
	}

	// Joint velocities
	joint_map.clear();
	nh_->getParam("initial_joint_velocities/joint_map", joint_map);
	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be recognized.
	if (nh_->hasParam("initial_joint_velocities/joint_map") && joint_map.size() == 0) {
		ROS_WARN_NAMED("mujoco", "Initial joint positions not recognized by rosparam server. Check your config, "
		                         "especially values for single axis joints should explicitly provided as string!");
	}
	for (auto const &[name, str_values] : joint_map) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Trying to set jointvels of joint " << name << " to values: " << str_values);
		int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			ROS_WARN_STREAM_NAMED("mujoco", "Joint with name '"
			                                    << name << "' could not be found. Initial joint velocity cannot be set!");
			continue;
		}
		int num_axes = 0;
		int jnt_type = model->jnt_type[id];
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
				break;
		}

		double axis_vals[num_axes];
		std::stringstream stream_values(str_values);
		int jnt_axis = 0;
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			if (jnt_axis > num_axes)
				break;
			axis_vals[jnt_axis] = std::stod(value);
			jnt_axis++;
		}

		if (jnt_axis != num_axes) {
			ROS_ERROR_STREAM_NAMED("mujoco", "Provided initial velocity values for joint "
			                                     << name << " don't match the degrees of freedom of the joint (exactly "
			                                     << num_axes << " values are needed)!");
			continue;
		}
		for (jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			setJointVelocity(model, data, axis_vals[jnt_axis], id, jnt_axis);
		}
	}
}

void setupCallbacks()
{
	service_servers_.push_back(nh_->advertiseService("set_pause", setPauseCB));
	service_servers_.push_back(nh_->advertiseService("shutdown", shutdownCB));
	service_servers_.push_back(nh_->advertiseService("reload", reloadCB));
	service_servers_.push_back(nh_->advertiseService("reset", resetCB));
	service_servers_.push_back(nh_->advertiseService("set_body_state", setBodyStateCB));
	service_servers_.push_back(nh_->advertiseService("get_body_state", getBodyStateCB));
	service_servers_.push_back(nh_->advertiseService("set_geom_properties", setGeomPropertiesCB));
	service_servers_.push_back(nh_->advertiseService("get_geom_properties", getGeomPropertiesCB));
	service_servers_.push_back(nh_->advertiseService("set_gravity", setGravityCB));
	service_servers_.push_back(nh_->advertiseService("get_gravity", getGravityCB));
	service_servers_.push_back(
	    nh_->advertiseService<mujoco_ros_msgs::GetStateUint::Request, mujoco_ros_msgs::GetStateUint::Response>(
	        "get_loadingrequest_state", [&](auto &request, auto &response) {
		        uint8_t status       = settings_.loadrequest.load();
		        response.state.value = status;

		        std::string description;
		        if (status == 0)
			        description = "Sim ready";
		        else if (status == 1)
			        description = "Loading in progress";
		        else if (status == 2)
			        description = "Loadingrequest issued";
		        response.state.description = description;
		        return true;
	        }));

	action_step_ =
	    std::make_unique<actionlib::SimpleActionServer<mujoco_ros_msgs::StepAction>>(*nh_, "step", onStepGoal, false);
	action_step_->start();

	mjcb_control = controlCallback;
	mjcb_passive = passiveCallback;
}

// Service call callbacks
bool shutdownCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	requestExternalShutdown();
	return true;
}

bool setPauseCB(mujoco_ros_msgs::SetPause::Request &req, mujoco_ros_msgs::SetPause::Response &resp)
{
	ROS_DEBUG_STREAM("PauseCB called with: " << (bool)req.paused);

	if (settings_.eval_mode && req.paused) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to pause simulation!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	settings_.run.store(!req.paused);
	if (settings_.run.load())
		settings_.manual_env_steps.store(0);
	resp.success = true;
	return true;
}

bool reloadCB(mujoco_ros_msgs::Reload::Request &req, mujoco_ros_msgs::Reload::Response &resp)
{
	ROS_DEBUG_NAMED("mujoco", "Requested reload via ROS service call");
	if (req.model != "") {
		ROS_DEBUG_NAMED("mujoco", "\tSupplied model to load as argument...");
		if (settings_.eval_mode) {
			ROS_DEBUG_NAMED("mujoco", "\tEvaluation mode is active. Checking has validity");
			if (settings_.admin_hash != req.admin_hash) {
				ROS_ERROR_NAMED("mujoco", "Insufficient permission to change model in eval mode!");
				resp.success        = false;
				resp.status_message = "Insufficient permission to change model in eval mode!";
				return true;
			}
			ROS_DEBUG_NAMED("mujoco", "\tHash valid, request authorized");
		}

		if (boost::filesystem::is_regular_file(req.model)) {
			boost::filesystem::path path(req.model);
			path    = boost::filesystem::absolute(path);
			int ret = mj_addFileVFS(&vfs_, path.parent_path().c_str(), path.filename().c_str());
			if (ret == 1) {
				ROS_ERROR_STREAM_NAMED(
				    "mujoco", "\tCould not save file "
				                  << req.model << " to VFS, because it's full! This should not happen, check with dev!");
				resp.success        = false;
				resp.status_message = "VFS error";
				return true;
			} else if (ret == 2) {
				ROS_ERROR_STREAM_NAMED("mujoco", "\tCould not save file "
				                                     << req.model
				                                     << " to VFS, because the file is already present in VFS!");
				resp.success        = false;
				resp.status_message = "VFS error";
				return true;
			}
			ROS_DEBUG_STREAM_NAMED("mujoco",
			                       "Successfully saved file " << req.model << " in mujoco VFS for accelerated loading");
			mju::strcpy_arr(filename_, req.model.c_str());
		} else { // File in memory
			ROS_DEBUG_STREAM_NAMED("mujoco", "\tProvided model is not a regular file. Treating string as content");
			mj_makeEmptyFileVFS(&vfs_, "rosparam_content", req.model.size() + 1);
			int file_idx = mj_findFileVFS(&vfs_, "rosparam_content");
			memcpy(vfs_.filedata[file_idx], req.model.c_str(), req.model.size() + 1);
			ROS_DEBUG_STREAM_NAMED("mujoco",
			                       "\tSuccessfully saved xml content as mujoco VFS file for accelerated loading");
			mju::strcpy_arr(filename_, "rosparam_content");
		}
	}
	settings_.loadrequest.store(2);
	resp.success = true;
	return true;
}

bool resetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	settings_.resetrequest.store(1);
	return true;
}

void onStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal)
{
	mujoco_ros_msgs::StepResult result;

	if (settings_.manual_env_steps.load() > 0 || settings_.run.load()) {
		ROS_WARN_NAMED("mujoco", "Simulation is currently unpaused. Stepping makes no sense right now.");
		result.success = false;
		action_step_->setPreempted(result);
		action_step_->setSucceeded(result);
		return;
	}

	mujoco_ros_msgs::StepFeedback feedback;

	feedback.steps_left = goal->num_steps + settings_.manual_env_steps.load();
	settings_.manual_env_steps.store(settings_.manual_env_steps.load() + goal->num_steps);

	result.success = true;
	while (settings_.manual_env_steps.load() > 0) {
		if (action_step_->isPreemptRequested() || !ros::ok() || settings_.exitrequest.load() > 0 ||
		    settings_.loadrequest.load() > 0 || settings_.resetrequest.load() > 0) {
			ROS_WARN_STREAM_NAMED("mujoco", "Simulation step action preempted");
			result.success = false;
			action_step_->setPreempted(result);
			settings_.manual_env_steps.store(0);
			break;
		}

		feedback.steps_left = settings_.manual_env_steps.load();
		action_step_->publishFeedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	feedback.steps_left = settings_.manual_env_steps.load();
	action_step_->publishFeedback(feedback);
	action_step_->setSucceeded(result);
}

bool setBodyStateCB(mujoco_ros_msgs::SetBodyState::Request &req, mujoco_ros_msgs::SetBodyState::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to set body state!");
			resp.success = false;
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to set body state!");
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.state.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);
	std::string full_error_msg("");
	resp.success = true;

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + std::to_string(env_id));
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int body_id = mj_name2id(env->model.get(), mjOBJ_BODY, req.state.name.c_str());
	if (body_id == -1) {
		ROS_WARN_STREAM_NAMED("mujoco", "Could not find model (mujoco body) with name " << req.state.name
		                                                                                << ". Trying to find geom...");
		int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.state.name.c_str());
		if (geom_id == -1) {
			std::string error_msg("Could not find model (not body nor geom) with name " + req.state.name);
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
			resp.success        = false;
			return true;
		}
		body_id = env->model->geom_bodyid[geom_id];
		ROS_WARN_STREAM_NAMED("mujoco", "found body named '" << mj_id2name(env->model.get(), mjOBJ_BODY, body_id)
		                                                     << "' as parent of geom '" << req.state.name << "'");
	}

	if (req.set_mass) {
		std::lock_guard<std::mutex> lk(sim_mtx);
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing mass '" << env->model->body_mass[body_id] << "' with new mass '"
		                                                      << req.state.mass << "'");
		env->model->body_mass[body_id] = req.state.mass;

		std::lock_guard<std::mutex> lk_render(render_mtx); // Prevent rendering the reset to q0
		mjtNum *qpos_tmp = mj_stackAlloc(env->data.get(), env->model->nq);
		mju_copy(qpos_tmp, env->data->qpos, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied current qpos state");
		mj_setConst(env->model.get(), env->data.get());
		ROS_DEBUG_NAMED("mujoco", "Reset constants because of mass change");
		mju_copy(env->data->qpos, qpos_tmp, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied qpos state back to data");
	}

	int jnt_adr     = env->model->body_jntadr[body_id];
	int jnt_type    = env->model->jnt_type[jnt_adr];
	int num_jnt     = env->model->body_jntnum[body_id];
	int jnt_qposadr = env->model->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = env->model->jnt_dofadr[jnt_adr];

	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::Twist target_twist;

	if (req.set_pose || req.set_twist || req.reset_qpos) {
		if (jnt_adr == -1) { // Check if body has joints
			std::string error_msg("Body has no joints, cannot move body!");
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			full_error_msg += error_msg + '\n';
			resp.success = false;
		} else if (jnt_type != mjJNT_FREE) { // Only freejoints can be moved
			std::string error_msg("Body " + req.state.name +
			                      " has no joint of type 'freetype'. This service call does not support any other types!");
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			full_error_msg += error_msg + '\n';
			resp.success = false;
		} else if (num_jnt > 1) {
			std::string error_msg("Body " + req.state.name + " has more than one joint ('" +
			                      std::to_string(env->model->body_jntnum[body_id]) +
			                      "'), pose/twist changes to bodies with more than one joint are not supported!");
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			full_error_msg += error_msg + '\n';
			resp.success = false;
		} else {
			// Lock mutex to prevent updating the body while a step is performed
			std::lock_guard<std::mutex> lk(sim_mtx);
			geometry_msgs::PoseStamped init_pose = req.state.pose;

			// Set freejoint position and quaternion
			if (req.set_pose && !req.reset_qpos) {
				bool valid_pose = true;
				if (req.state.pose.header.frame_id != "" && req.state.pose.header.frame_id != "world") {
					try {
						tf_bufferPtr_->transform<geometry_msgs::PoseStamped>(req.state.pose, target_pose, "world");
					} catch (tf2::TransformException &ex) {
						ROS_WARN_STREAM_NAMED("mujoco", ex.what());
						full_error_msg +=
						    "Could not transform frame '" + req.state.pose.header.frame_id + "' to frame world" + '\n';
						resp.success = false;
						valid_pose   = false;
					}
				} else {
					target_pose = req.state.pose;
				}

				if (valid_pose) {
					mjtNum quat[4] = { target_pose.pose.orientation.w, target_pose.pose.orientation.x,
						                target_pose.pose.orientation.y, target_pose.pose.orientation.z };
					mju_normalize4(quat);

					ROS_DEBUG_STREAM_NAMED("mujoco", "Setting body pose to " << target_pose.pose.position.x << ", "
					                                                         << target_pose.pose.position.y << ", "
					                                                         << target_pose.pose.position.z << ", "
					                                                         << quat[0] << ", " << quat[1] << ", " << quat[2]
					                                                         << ", " << quat[3] << " (xyz wxyz)");

					env->data->qpos[jnt_qposadr]     = target_pose.pose.position.x;
					env->data->qpos[jnt_qposadr + 1] = target_pose.pose.position.y;
					env->data->qpos[jnt_qposadr + 2] = target_pose.pose.position.z;
					env->data->qpos[jnt_qposadr + 3] = quat[0];
					env->data->qpos[jnt_qposadr + 4] = quat[1];
					env->data->qpos[jnt_qposadr + 5] = quat[2];
					env->data->qpos[jnt_qposadr + 6] = quat[3];
				}
			}

			if (req.reset_qpos && num_jnt > 0) {
				int num_dofs = 7; // Is always 7 because the joint is restricted to one joint of type freejoint
				ROS_WARN_COND_NAMED(req.set_pose, "mujoco",
				                    "set_pose and reset_qpos were both passed. reset_qpos will overwrite the custom pose!");
				ROS_DEBUG_NAMED("mujoco", "Resetting body qpos");
				mju_copy(env->data->qpos + env->model->jnt_qposadr[jnt_adr],
				         env->model->qpos0 + env->model->jnt_qposadr[jnt_adr], num_dofs);
				if (!req.set_twist) {
					// Reset twist if no desired twist is given (default twist is 0 0 0 0 0 0)
					req.set_twist   = true;
					req.state.twist = geometry_msgs::TwistStamped();
				}
			}
			// Set freejoint twist
			if (req.set_twist) {
				// Only pose can be transformed. Twist will be ignored!
				if (req.state.twist.header.frame_id != "" && req.state.twist.header.frame_id != "world") {
					std::string error_msg("Transforming twists from other frames is not supported! Not setting twist.");
					ROS_WARN_STREAM_COND_NAMED(req.state.twist.header.frame_id != "" &&
					                               req.state.twist.header.frame_id != "world",
					                           "mujoco", error_msg);
					full_error_msg += error_msg + '\n';
					resp.success = false;
				} else {
					ROS_DEBUG_STREAM_NAMED("mujoco",
					                       "Setting body twist to "
					                           << req.state.twist.twist.linear.x << ", " << req.state.twist.twist.linear.y
					                           << ", " << req.state.twist.twist.linear.z << ", "
					                           << req.state.twist.twist.angular.x << ", " << req.state.twist.twist.angular.y
					                           << ", " << req.state.twist.twist.angular.z << " (xyz rpy)");
					env->data->qvel[jnt_dofadr]     = req.state.twist.twist.linear.x;
					env->data->qvel[jnt_dofadr + 1] = req.state.twist.twist.linear.y;
					env->data->qvel[jnt_dofadr + 2] = req.state.twist.twist.linear.z;
					env->data->qvel[jnt_dofadr + 3] = req.state.twist.twist.angular.x;
					env->data->qvel[jnt_dofadr + 4] = req.state.twist.twist.angular.y;
					env->data->qvel[jnt_dofadr + 5] = req.state.twist.twist.angular.z;
				}
			}
		}
	}

	resp.status_message = static_cast<decltype(resp.status_message)>(full_error_msg);
	return true;
}

bool getBodyStateCB(mujoco_ros_msgs::GetBodyState::Request &req, mujoco_ros_msgs::GetBodyState::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to get body state!");
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to get body state!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);
	resp.success     = true;

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + std::to_string(env_id));
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int body_id = mj_name2id(env->model.get(), mjOBJ_BODY, req.name.c_str());
	if (body_id == -1) {
		ROS_WARN_STREAM_NAMED("mujoco",
		                      "Could not find model (mujoco body) with name " << req.name << ". Trying to find geom...");
		int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.name.c_str());
		if (geom_id == -1) {
			std::string error_msg("Could not find model (not body nor geom) with name " + req.name);
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
			resp.success        = false;
			return true;
		}
		body_id = env->model->geom_bodyid[geom_id];
		ROS_WARN_STREAM_NAMED("mujoco", "found body named '" << mj_id2name(env->model.get(), mjOBJ_BODY, body_id)
		                                                     << "' as parent of geom '" << req.name << "'");
	}

	resp.state.name = mj_id2name(env->model.get(), mjOBJ_BODY, body_id);
	resp.state.mass = env->model->body_mass[body_id];

	int jnt_adr     = env->model->body_jntadr[body_id];
	int jnt_type    = env->model->jnt_type[jnt_adr];
	int num_jnt     = env->model->body_jntnum[body_id];
	int jnt_qposadr = env->model->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = env->model->jnt_dofadr[jnt_adr];

	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::Twist target_twist;

	// Stop sim to get data out of the same point in time
	std::lock_guard<std::mutex> lk(sim_mtx);
	if (jnt_adr == -1 || jnt_type != mjJNT_FREE || num_jnt > 1) {
		resp.state.pose.header             = std_msgs::Header();
		resp.state.pose.header.frame_id    = "world";
		resp.state.pose.pose.position.x    = env->data->xpos[body_id * 3];
		resp.state.pose.pose.position.y    = env->data->xpos[body_id * 3 + 1];
		resp.state.pose.pose.position.z    = env->data->xpos[body_id * 3 + 2];
		resp.state.pose.pose.orientation.w = env->data->xquat[body_id * 3];
		resp.state.pose.pose.orientation.x = env->data->xquat[body_id * 3 + 1];
		resp.state.pose.pose.orientation.y = env->data->xquat[body_id * 3 + 2];
		resp.state.pose.pose.orientation.z = env->data->xquat[body_id * 3 + 3];

		resp.state.twist.header          = std_msgs::Header();
		resp.state.twist.header.frame_id = "world";
		resp.state.twist.twist.linear.x  = env->data->cvel[body_id * 6];
		resp.state.twist.twist.linear.y  = env->data->cvel[body_id * 6 + 1];
		resp.state.twist.twist.linear.z  = env->data->cvel[body_id * 6 + 2];
		resp.state.twist.twist.angular.x = env->data->cvel[body_id * 6 + 3];
		resp.state.twist.twist.angular.y = env->data->cvel[body_id * 6 + 4];
		resp.state.twist.twist.angular.z = env->data->cvel[body_id * 6 + 5];
	} else {
		resp.state.pose.header             = std_msgs::Header();
		resp.state.pose.header.frame_id    = "world";
		resp.state.pose.pose.position.x    = env->data->qpos[jnt_qposadr];
		resp.state.pose.pose.position.y    = env->data->qpos[jnt_qposadr + 1];
		resp.state.pose.pose.position.z    = env->data->qpos[jnt_qposadr + 2];
		resp.state.pose.pose.orientation.w = env->data->qpos[jnt_qposadr + 3];
		resp.state.pose.pose.orientation.x = env->data->qpos[jnt_qposadr + 4];
		resp.state.pose.pose.orientation.y = env->data->qpos[jnt_qposadr + 5];
		resp.state.pose.pose.orientation.z = env->data->qpos[jnt_qposadr + 6];

		resp.state.twist.header          = std_msgs::Header();
		resp.state.twist.header.frame_id = "world";
		resp.state.twist.twist.linear.x  = env->data->qvel[jnt_dofadr];
		resp.state.twist.twist.linear.y  = env->data->qvel[jnt_dofadr + 1];
		resp.state.twist.twist.linear.z  = env->data->qvel[jnt_dofadr + 2];
		resp.state.twist.twist.angular.x = env->data->qvel[jnt_dofadr + 3];
		resp.state.twist.twist.angular.y = env->data->qvel[jnt_dofadr + 4];
		resp.state.twist.twist.angular.z = env->data->qvel[jnt_dofadr + 5];
	}

	return true;
}

bool setGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
                         mujoco_ros_msgs::SetGeomProperties::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to set geom properties!");
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to set geom properties!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.properties.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + env_id);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.properties.name.c_str());
	if (geom_id == -1) {
		std::string error_msg("Could not find model (mujoco geom) with name " + req.properties.name);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int body_id = env->model->geom_bodyid[geom_id];

	// Lock mutex to prevent updating the body while a step is performed
	std::lock_guard<std::mutex> lk(sim_mtx);

	ROS_DEBUG_STREAM_NAMED("mujoco", "Changing properties of geom '" << req.properties.name.c_str() << "' ...");
	if (req.set_mass) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing mass '" << env->model->body_mass[body_id] << "' with new mass '"
		                                                      << req.properties.body_mass << "'");
		env->model->body_mass[body_id] = req.properties.body_mass;
	}
	if (req.set_friction) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing friction '" << env->model->geom_friction[geom_id * 3] << ", "
		                                                          << env->model->geom_friction[geom_id * 3 + 1] << ", "
		                                                          << env->model->geom_friction[geom_id * 3 + 2]
		                                                          << "' with new mass '" << req.properties.friction_slide
		                                                          << ", " << req.properties.friction_spin << ", "
		                                                          << req.properties.friction_roll << "'");
		env->model->geom_friction[geom_id * 3]     = req.properties.friction_slide;
		env->model->geom_friction[geom_id * 3 + 1] = req.properties.friction_spin;
		env->model->geom_friction[geom_id * 3 + 2] = req.properties.friction_roll;
	}
	if (req.set_type) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing type '" << env->model->geom_type[geom_id] << "' with new type '"
		                                                      << req.properties.type << "'");
		env->model->geom_type[geom_id] = req.properties.type.value;
	}

	if (req.set_size) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing size '" << env->model->geom_size[geom_id * 3] << ", "
		                                                      << env->model->geom_size[geom_id * 3 + 1] << ", "
		                                                      << env->model->geom_size[geom_id * 3 + 2]
		                                                      << "' with new size '" << req.properties.size_0 << ", "
		                                                      << req.properties.size_1 << ", " << req.properties.size_2
		                                                      << "'");
		env->model->geom_size[geom_id * 3]     = req.properties.size_0;
		env->model->geom_size[geom_id * 3 + 1] = req.properties.size_1;
		env->model->geom_size[geom_id * 3 + 2] = req.properties.size_2;

		mj_forward(env->model.get(), env->data.get());
	}

	if (req.set_type || req.set_mass) {
		std::lock_guard<std::mutex> lk(render_mtx); // Prevent rendering the reset to q0

		mjtNum *qpos_tmp = mj_stackAlloc(env->data.get(), env->model->nq);
		mju_copy(qpos_tmp, env->data->qpos, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied current qpos state");
		mj_setConst(env->model.get(), env->data.get());
		ROS_DEBUG_NAMED("mujoco", "Reset constants");
		mju_copy(env->data->qpos, qpos_tmp, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied qpos state back to data");
	}

	env->notifyGeomChanged(geom_id);

	resp.success = true;
	return true;
}

bool getGeomPropertiesCB(mujoco_ros_msgs::GetGeomProperties::Request &req,
                         mujoco_ros_msgs::GetGeomProperties::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to get geom properties!");
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to get geom properties!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + env_id);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.geom_name.c_str());
	if (geom_id == -1) {
		std::string error_msg("Could not find model (mujoco geom) with name " + req.geom_name);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	int body_id = env->model->geom_bodyid[geom_id];

	// Lock mutex to get data within one step
	std::lock_guard<std::mutex> lk(sim_mtx);
	resp.properties.name           = req.geom_name;
	resp.properties.body_mass      = env->model->body_mass[body_id];
	resp.properties.friction_slide = env->model->geom_friction[geom_id * 3];
	resp.properties.friction_spin  = env->model->geom_friction[geom_id * 3 + 1];
	resp.properties.friction_roll  = env->model->geom_friction[geom_id * 3 + 2];

	resp.properties.type.value = env->model->geom_type[geom_id];

	resp.properties.size_0 = env->model->geom_size[geom_id * 3];
	resp.properties.size_1 = env->model->geom_size[geom_id * 3 + 1];
	resp.properties.size_2 = env->model->geom_size[geom_id * 3 + 2];

	resp.success = true;
	return true;
}

bool setGravityCB(mujoco_ros_msgs::SetGravity::Request &req, mujoco_ros_msgs::SetGravity::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to set gravity!");
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to set gravity!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + env_id);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	// Lock mutex to get data within one step
	std::lock_guard<std::mutex> lk(sim_mtx);
	for (int i = 0; i < 3; ++i) {
		env->model->opt.gravity[i] = req.gravity[i];
	}
	resp.success = true;
	return true;
}

bool getGravityCB(mujoco_ros_msgs::GetGravity::Request &req, mujoco_ros_msgs::GetGravity::Response &resp)
{
	if (settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to get gravity!");
			resp.status_message =
			    static_cast<decltype(resp.status_message)>("Hash mismatch, no permission to get gravity!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg("Could not find environment with id " + env_id);
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return true;
	}

	// Lock mutex to get data within one step
	std::lock_guard<std::mutex> lk(sim_mtx);
	for (int i = 0; i < 3; ++i) {
		resp.gravity[i] = env->model->opt.gravity[i];
	}
	resp.success = true;
	return true;
}

} // end namespace detail

} // end namespace MujocoSim
