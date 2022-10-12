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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <rosgraph_msgs/Clock.h>
#include <boost/filesystem.hpp>

#include <iostream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace MujocoSim {

using namespace detail;

MujocoSim::MujocoEnvPtr MujocoSim::detail::unit_testing::getmjEnv()
{
	return main_env_;
}

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

void renderCallback(mjData *data, mjvScene *scene)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runRenderCbs(scene);
	}
}

void lastStageCallback(mjData *data)
{
	MujocoEnvPtr env = environments::getEnv(data);
	if (env) {
		env->runLastStageCbs();
	}
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

void init(std::string modelfile)
{
	nh_.reset(new ros::NodeHandle("~"));

	settings_.exitrequest = 0;

	// Print version, check compatibility
	ROS_INFO("MuJoCo Pro library version %.2lf\n", 0.01 * mj_version());
	if (mjVERSION_HEADER != mj_version()) {
		ROS_ERROR_NAMED("mujoco", "Headers and library have different versions");
		mju_error("Headers and library have different versions");
	}

	// Check which sim mode to use
	nh_->param<int>("num_simulations", num_simulations_, 1);
	env_list_.resize(num_simulations_);
	if (num_simulations_ > 1) {
		sim_mode_      = simMode::PARALLEL;
		ready_threads_ = 0;
	} else {
		sim_mode_ = simMode::SINGLE;
	}

	double time = ros::Time::now().toSec();
	if (time > 0) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "ROS time was not 0, starting with time at " << time << " seconds");
		last_time_ = time;
	}

	nh_->param<bool>("benchmark_time", benchmark_env_time_, false);

	bool unpause;
	nh_->param<bool>("unpause", unpause, true);
	if (sim_mode_ == simMode::SINGLE) {
		std::string main_env_namespace;
		nh_->param<std::string>("ns", main_env_namespace, "/");
		main_env_.reset(new MujocoEnv(main_env_namespace));

		if (unpause) {
			settings_.run = 1;
		} else {
			ROS_DEBUG_NAMED("mujoco", "Starting in paused state");
			settings_.run = 0;
		}
	} else {
		settings_.run = 0;
		ROS_DEBUG_NAMED("mujoco", "Multi env sim mode starting in mandatory pause mode.");

		std::string bootstrap_launchfile;
		std::vector<std::string> bootstrap_launchargs;

		nh_->param<std::string>("bootstrap_launchfile", bootstrap_launchfile, "");
		nh_->param<std::vector<std::string>>("bootstrap_launchargs", bootstrap_launchargs, std::vector<std::string>());

		if (!bootstrap_launchfile.empty()) {
			ROS_INFO_STREAM_NAMED("mujoco",
			                      "Using '" << bootstrap_launchfile << "' as environment bootstrapping launchfile");
			if (!bootstrap_launchargs.empty()) {
				std::string bootstrap_launchargs_string;
				for (const auto &s : bootstrap_launchargs) {
					bootstrap_launchargs_string += s + " ";
				}
				ROS_DEBUG_STREAM_NAMED("mujoco", "Got launchargs: " << bootstrap_launchargs_string);
			} else {
				ROS_DEBUG_NAMED("mujoco", "No bootstrap args provided");
			}
		} else {
			ROS_WARN_NAMED("mujoco", "No bootstrapping launchfile provided, was this on purpose?");
		}

		MujocoEnvParallelPtr env_ptr;
		for (int i = 0; i < num_simulations_; i++) {
			// Creation of envs in the init is better to avoid destruction and re-creation of envs in the loading function.
			bootstrap_launchargs.push_back("ns:=env" + std::to_string(i));
			env_ptr.reset(new MujocoEnvParallel("env" + std::to_string(i), bootstrap_launchfile, bootstrap_launchargs));
			bootstrap_launchargs.pop_back();
			env_list_[i] = env_ptr;
			if (i == 0)
				main_env_ = env_ptr;
		}
	}

	nh_->param<bool>("visualize", vis_, true);

	if (vis_) {
		initVisual();
	} else {
		ROS_DEBUG_NAMED("mujoco", "Will run in headless mode!");
	}

	pub_clock_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 10);

	// If not already set, set use_sim_time param manually
	if (!(nh_->hasParam("/use_sim_time")))
		nh_->setParam("/use_sim_time", true);

	if (!modelfile.empty()) {
		std::strcpy(filename_, modelfile.c_str());
		settings_.loadrequest = 2;
	}

	setupCallbacks();

	std::thread simthread(simulate);

	tf_bufferPtr_.reset(new tf2_ros::Buffer());
	tf_bufferPtr_->setUsingDedicatedThread(true);
	tf_listenerPtr_.reset(new tf2_ros::TransformListener(*tf_bufferPtr_));

	eventloop();

	ROS_DEBUG_NAMED("mujoco", "Event loop terminated");
	settings_.exitrequest = 1;
	simthread.join();

	if (sim_mode_ == simMode::PARALLEL) {
		step_signal_.notify_all();
		for (const auto &env : env_list_) {
			MujocoEnvParallelPtr parallel_env = boost::static_pointer_cast<MujocoEnvParallel>(env);
			if (parallel_env->loop_thread != nullptr) {
				parallel_env->loop_thread->join();
				ROS_DEBUG_STREAM_NAMED("mujoco", "Joined loop thread of " << env->name);
			}
		}
	}

	if (vis_)
		uiClearCallback(window_);

	ROS_DEBUG_NAMED("mujoco", "Sim thread terminated");

	sim_mtx.unlock();
	render_mtx.unlock();

	main_env_.reset();
	plugin_utils::unloadPluginloader();
	mjv_freeScene(&scn_);
	mjr_freeContext(&con_);

	for (auto ss : service_servers_) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Shutting down service " << ss.getService());
		ss.shutdown();
	}
	service_servers_.clear();
	action_step_->shutdown();

	ROS_DEBUG_NAMED("mujoco", "Cleanup done");
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
	settings_.exitrequest = 1;
}

void resetSim()
{
	if (main_env_->model) {
		mj_resetData(main_env_->model.get(), main_env_->data.get());
		if (last_time_ > 0)
			main_env_->data->time = last_time_;
		loadInitialJointStates(main_env_->model, main_env_->data);
		main_env_->reset();
		mj_forward(main_env_->model.get(), main_env_->data.get());
		publishSimTime(main_env_->data->time);
		profilerUpdate(main_env_->model, main_env_->data);
		sensorUpdate(main_env_->model, main_env_->data);
		updateSettings(main_env_->model);
	}
}

void synchedMultiSimStep()
{
	publishSimTime(main_env_->data->time);

	std::chrono::_V2::system_clock::time_point t0, t1;

	if (benchmark_env_time_)
		t0 = std::chrono::high_resolution_clock::now();

	bool waiting = true;
	uint trys    = 1;

	while (waiting && trys < 1000 && !settings_.exitrequest) {
		{
			std::unique_lock<std::mutex> lk(readyness_mtx);
			waiting = ready_threads_ != num_simulations_;

			if (not waiting || settings_.exitrequest) {
				step_signal_.notify_all();
				ready_threads_ -= num_simulations_;
			}
			trys += 1;
		}
	}

	// Publish new synchronized sim time
	publishSimTime(main_env_->data->time);

	if (benchmark_env_time_) {
		t1             = std::chrono::high_resolution_clock::now();
		uint curr_step = (main_env_->data->time / main_env_->model->opt.timestep);

		std::chrono::duration<double, std::milli> ms_double = t1 - t0;

		ROS_DEBUG_STREAM_COND_NAMED(curr_step % 100 == 0 || curr_step == 1, "mujoco",
		                            "One step with " << num_simulations_ << " instances took " << ms_double.count()
		                                             << " ms ");
	}
}

namespace detail {

void publishSimTime(mjtNum time)
{
	rosgraph_msgs::Clock ros_time;
	ros_time.clock.fromSec(time);
	pub_clock_.publish(ros_time);
	last_time_ = time;
}

void eventloop(void)
{
	while (ros::ok() &&
	       ((!settings_.exitrequest && !vis_) || (!settings_.exitrequest && !glfwWindowShouldClose(window_)))) {
		// Critical operations
		sim_mtx.lock();

		if (settings_.loadrequest == 1) {
			loadModel();
		} else if (settings_.loadrequest > 1) {
			settings_.loadrequest = 1;
		}

		if (vis_) {
			// Handle events (via callbacks)
			glfwPollEvents();

			// Prepare to render
			prepare(main_env_->model, main_env_->data);
		}

		// Allow simulation thread to run
		sim_mtx.unlock();

		// render while sim is running
		if (vis_)
			render(window_);
	}
}

void simulate(void)
{
	int num_steps;
	nh_->param<int>("num_steps", num_steps, -1);

	ROS_DEBUG_STREAM_COND_NAMED(num_steps > 0, "mujoco", "Simulation will terminate after " << num_steps << " steps");

	// cpu-sim syncronization point
	double cpusync = 0;
	mjtNum simsync = 0;

	mjModelPtr model;
	mjDataPtr data;

	while (!settings_.exitrequest && num_steps != 0) {
		model = main_env_->model;
		data  = main_env_->data;

		if (data) {
			publishSimTime(data->time);
		}

		if (settings_.run && settings_.busywait) {
			std::this_thread::yield();
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		sim_mtx.lock();

		if (model) {
			if (settings_.run && sim_mode_ == simMode::SINGLE) {
				double tmstart = glfwGetTime();

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

				// Out-of-sync (for any reason)
				mjtNum offset = mju_abs((data->time * settings_.slow_down - simsync) - (tmstart - cpusync));
				if (data->time * settings_.slow_down < simsync || tmstart < cpusync || cpusync == 0 ||
				    offset > syncmisalign_ * settings_.slow_down || settings_.speed_changed) {
					// Re-sync

					// ROS_WARN_STREAM_NAMED("mujoco", "Out of sync by " << offset << ". Re-syncing...");

					cpusync                 = tmstart;
					simsync                 = data->time * settings_.slow_down;
					settings_.speed_changed = false;

					// Clear old perturbations, apply new
					mju_zero(data->xfrc_applied, 6 * model->nbody);
					mjv_applyPerturbPose(model.get(), data.get(), &pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(model.get(), data.get(), &pert_);

					// Run single step, let next iteration deal with timing
					mj_step(model.get(), data.get());
					publishSimTime(data->time);
					lastStageCallback(data.get());

					// Count steps until termination
					if (num_steps > 0) {
						num_steps--;
						ROS_INFO_COND_NAMED(num_steps == 0, "mujoco", "running last sim step before termination!");
					}
				} else { // in-sync
					// Step while simtime lags behind cputime , and within safefactor
					while ((data->time * settings_.slow_down - simsync) < (glfwGetTime() - cpusync) &&
					       (glfwGetTime() - tmstart) < refreshfactor_ / vmode_.refreshRate &&
					       (num_steps == -1 || num_steps > 0)) {
						// clear old perturbations, apply new
						mju_zero(data->xfrc_applied, 6 * model->nbody);
						mjv_applyPerturbPose(model.get(), data.get(), &pert_, 0); // Move mocap bodies only
						mjv_applyPerturbForce(model.get(), data.get(), &pert_);

						// Run mj_step
						mjtNum prevtm = data->time * settings_.slow_down;
						mj_step(model.get(), data.get());
						publishSimTime(data->time);
						lastStageCallback(data.get());

						// Count steps until termination
						if (num_steps > 0) {
							num_steps--;
							ROS_INFO_COND_NAMED(num_steps == 0, "mujoco", "running last sim step before termination!");
						}

						// break on reset
						if (data->time * settings_.slow_down < prevtm) {
							break;
						}
					}
				}
			} else if (sim_mode_ == simMode::PARALLEL) {
				if (settings_.manual_env_steps != 0) {
					// clear old perturbations, apply new
					mju_zero(data->xfrc_applied, 6 * model->nbody);
					mjv_applyPerturbPose(model.get(), data.get(), &pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(model.get(), data.get(), &pert_);

					synchedMultiSimStep();
					settings_.manual_env_steps--;
				} else { // Paused in PARALLEL mode
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			} else { // Paused in single env mode
				if (settings_.manual_env_steps > 0) { // Action call or arrow keys used for stepping
					mju_zero(data->xfrc_applied, 6 * model->nbody);
					mjv_applyPerturbPose(model.get(), data.get(), &pert_, 0); // Move mocap bodies only
					mjv_applyPerturbForce(model.get(), data.get(), &pert_);

					// Run single step
					mj_step(model.get(), data.get());
					publishSimTime(data->time);
					lastStageCallback(data.get());

					settings_.manual_env_steps--;
				} else {
					mjv_applyPerturbPose(model.get(), data.get(), &pert_, 1); // Move mocap and dynamic bodies

					// Run mj_forward, to update rendering and joint sliders
					mj_forward(model.get(), data.get());
				}
			}
		}
		sim_mtx.unlock();

		std::string modelfile;
		nh_->getParam("modelfile", modelfile);
		if (!modelfile.empty() && strcmp(filename_, modelfile.c_str())) {
			ROS_DEBUG_STREAM_NAMED("mujoco", "Got new modelfile from param server! Requesting load ...");
			std::strcpy(filename_, modelfile.c_str());
			settings_.loadrequest = 2;
		}
	}
	// Requests eventloop shutdown in case we ran out of simulation steps to use
	settings_.exitrequest = 1;
}

void envStepLoop(MujocoEnvParallelPtr env)
{
	while (!settings_.exitrequest && not env->stop_loop) {
		if (settings_.ctrlnoisestd) {
			// Convert rate and scale to discrete time given current timestep
			mjtNum rate  = mju_exp(-env->model->opt.timestep / settings_.ctrlnoiserate);
			mjtNum scale = settings_.ctrlnoisestd * mju_sqrt(1 - rate * rate);

			for (int i = 0; i < env->model->nu; i++) {
				// Update noise
				env->ctrlnoise[i] = rate * env->ctrlnoise[i] + scale * mju_standardNormal(nullptr);
				// Apply noise
				env->data->ctrl[i] = env->ctrlnoise[i];
			}
		}

		{
			// ROS_DEBUG_STREAM_NAMED("envStepLoop", "Increasing number of ready threads to " << ready_threads_+1 << " ["
			// << env->name << "]"); ROS_DEBUG_STREAM_NAMED("envStepLoop", "Waiting for signal... [" << env->name << "]");
			std::unique_lock<std::mutex> lk(readyness_mtx);
			ready_threads_ += 1;
			step_signal_.wait(lk);
		}
		if (env->stop_loop || settings_.exitrequest)
			break;
		// Run mj_step
		mj_step(env->model.get(), env->data.get());
		lastStageCallback(env->data.get());
	}
	ROS_DEBUG_STREAM_COND_NAMED(settings_.exitrequest, "envStepLoop",
	                            "halted because of exit request [" << env->name << "]");
}

void initVisual()
{
	if (!glfwInit()) {
		ROS_ERROR_NAMED("mujoco", "Could not initialize GLFW");
		mju_error("Could not initialize GLFW");
	}
	mjcb_time = timer;

	// multisampling
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_VISIBLE, 1);

	// get videomode and save
	vmode_ = *glfwGetVideoMode(glfwGetPrimaryMonitor());

	// create window
	window_ = glfwCreateWindow((2 * vmode_.width) / 3, (2 * vmode_.height) / 3, "Simulate", NULL, NULL);

	if (!window_) {
		glfwTerminate();
		ROS_ERROR_NAMED("mujoco", "Could not create window");
		mju_error("could not create window");
	}

	// save window position and size
	glfwGetWindowPos(window_, windowpos_, windowpos_ + 1);
	glfwGetWindowSize(window_, windowsize_, windowsize_ + 1);

	// make context current, set v-sync
	glfwMakeContextCurrent(window_);
	glfwSwapInterval(settings_.vsync);

	// init abstract visualization
	mjv_defaultCamera(&cam_);
	mjv_defaultOption(&vopt_);
	profilerInit();
	sensorInit();

	// make empty scene
	mjv_defaultScene(&scn_);
	mjv_makeScene(NULL, &scn_, maxgeom_);

	// select default font
	int fontscale  = uiFontScale(window_);
	settings_.font = fontscale / 50 - 1;

	// make empty context
	mjr_defaultContext(&con_);
	mjr_makeContext(NULL, &con_, fontscale);

	// set GLFW callbacks
	uiSetCallback(window_, &uistate_, uiEvent, uiLayout);
	glfwSetWindowRefreshCallback(window_, render);

	// init state und uis
	std::memset(&uistate_, 0, sizeof(mjuiState));
	std::memset(&ui0_, 0, sizeof(mjUI));
	std::memset(&ui1_, 0, sizeof(mjUI));
	ui0_.spacing   = mjui_themeSpacing(settings_.spacing);
	ui0_.color     = mjui_themeColor(settings_.color);
	ui0_.predicate = uiPredicate;
	ui0_.rectid    = 1;
	ui0_.auxid     = 0;

	ui1_.spacing   = mjui_themeSpacing(settings_.spacing);
	ui1_.color     = mjui_themeColor(settings_.color);
	ui1_.predicate = uiPredicate;
	ui1_.rectid    = 2;
	ui1_.auxid     = 1;

	// populate uis with standard sections
	mjui_add(&ui0_, defFile);
	mjui_add(&ui0_, defOption);
	mjui_add(&ui0_, defSimulation);
	mjui_add(&ui0_, defWatch);
	uiModify(window_, &ui0_, &uistate_, &con_);
	uiModify(window_, &ui1_, &uistate_, &con_);
}

void loadModel(void)
{
	settings_.loadrequest = 0;
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
		ROS_WARN_NAMED("mujoco", "Model compiled, but simulation warning (paused): \n %s\n\n", error);
		std::printf("Model compiled, but simulation warning (paused): \n %s\n\n", error);
		settings_.run = 0;
	}

	if (sim_mode_ == simMode::SINGLE) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Setting up env '" << main_env_->name << "' ...");
		// Delete old model, assign new
		main_env_->model.reset(mnew);
		environments::assignData(mj_makeData(mnew), main_env_);
		setupEnv(main_env_);
	} else {
		for (const auto &env : env_list_) {
			ROS_DEBUG_STREAM_NAMED("mujoco", "Setting up env '" << env->name << "' ...");
			MujocoEnvParallelPtr parallel_env = boost::static_pointer_cast<MujocoEnvParallel>(env);
			if (parallel_env->loop_thread != nullptr) {
				parallel_env->stop_loop = true;
				parallel_env->loop_thread->join();
			}
			env->model.reset(mnew);
			environments::assignData(mj_makeData(mnew), env);
			setupEnv(env);
			parallel_env->stop_loop = false;
			ROS_DEBUG_STREAM_NAMED("mujoco", "Starting thread for " << parallel_env->name);
			parallel_env->loop_thread = new std::thread(envStepLoop, parallel_env);
		}
	}

	ROS_DEBUG_NAMED("mujoco", "clear perturb ...");
	// Clear perturbation state
	pert_.active     = 0;
	pert_.select     = 0;
	pert_.skinselect = -1;

	if (vis_)
		mjr_makeContext(main_env_->model.get(), &con_, 50 * (settings_.font + 1));

	// Align and scale view unless reloading the same file
	if (mju::strcmp_arr(filename_, previous_filename_)) {
		alignScale(main_env_->model);
		mju::strcpy_arr(previous_filename_, filename_);
	}

	ROS_DEBUG_NAMED("mujoco", "updating scene...");
	// Update scene
	mjv_updateScene(main_env_->model.get(), main_env_->data.get(), &vopt_, &pert_, &cam_, mjCAT_ALL, &scn_);

	if (vis_) {
		// Set window title to model name
		if (window_ && main_env_->model->names) {
			char title[200] = "Simulate : ";
			mju::strcat_arr(title, main_env_->model->names);
			glfwSetWindowTitle(window_, title);
		}

		// Set keyframe range and divisions
		ui0_.sect[SECT_SIMULATION].item[5].slider.range[0]  = 0;
		ui0_.sect[SECT_SIMULATION].item[5].slider.range[1]  = mjMAX(0, main_env_->model->nkey - 1);
		ui0_.sect[SECT_SIMULATION].item[5].slider.divisions = mjMAX(1, main_env_->model->nkey - 1);

		// Rebuild UI Sections
		makeSections();

		// Full UI update
		uiModify(window_, &ui0_, &uistate_, &con_);
		uiModify(window_, &ui1_, &uistate_, &con_);
	}

	ROS_DEBUG_NAMED("mujoco", "updating settings ...");
	// model pointers of all envs point to the same model instance
	updateSettings(main_env_->model);
	ROS_DEBUG_NAMED("mujoco", "settings updated ...");
}

void setupEnv(MujocoEnvPtr env)
{
	// if time == 0, then sim is loaded for the first time
	if (last_time_ > 0)
		env->data->time = last_time_;

	loadInitialJointStates(env->model, env->data);

	mj_forward(env->model.get(), env->data.get());

	publishSimTime(env->data->time);

	ROS_DEBUG_NAMED("mujoco", "resetting noise ...");
	// Allocate ctrlnoise
	free(env->ctrlnoise);
	env->ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * env->model->nu);
	mju_zero(env->ctrlnoise, env->model->nu);

	ROS_DEBUG_NAMED("mujoco", "creating scene ...");
	// Re-create scene and context
	mjv_makeScene(env->model.get(), &scn_, maxgeom_);

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

int uiPredicate(int category, void *userdata)
{
	switch (category) {
		case 2: // require model
			return (main_env_->model != NULL);

		case 3: //
			return (main_env_->model && main_env_->model->nkey);

		case 4:
			return (main_env_->model && !settings_.run);

		default:
			return 1;
	}
}

void render(GLFWwindow *window)
{
	render_mtx.lock();

	// get 3D rectangle and reduced for profiler
	mjrRect rect      = uistate_.rect[3];
	mjrRect smallrect = rect;

	if (settings_.profiler) {
		smallrect.width = rect.width - rect.width / 4;
	}

	// no model
	if (!main_env_->model) {
		// blank screen
		mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);

		// label
		if (settings_.loadrequest)
			mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, "loading", NULL, &con_);

		//// We don't want this. A model should be loaded over services or during start
		// else
		// mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con_);

		// render uis
		if (settings_.ui0)
			mjui_render(&ui0_, &uistate_, &con_);
		if (settings_.ui1)
			mjui_render(&ui1_, &uistate_, &con_);

		// finalize
		glfwSwapBuffers(window);
	} else {
		renderCallback(main_env_->data.get(), &scn_);
	}
	// render scene
	mjr_render(rect, &scn_, &con_);

	// show pause/loading label
	if (!settings_.run || settings_.loadrequest)
		mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, settings_.loadrequest ? "loading" : "pause", NULL, &con_);

	// show realtime label
	if (settings_.run && settings_.slow_down != 1) {
		std::string realtime_label = "1/" + std::to_string(settings_.slow_down) + "x";
		mjr_overlay(mjFONT_BIG, mjGRID_TOPRIGHT, smallrect, realtime_label.c_str(), NULL, &con_);
	}

	// show ui 0
	if (settings_.ui0)
		mjui_render(&ui0_, &uistate_, &con_);

	// show ui 1
	if (settings_.ui1)
		mjui_render(&ui1_, &uistate_, &con_);

	// show help
	if (settings_.help)
		mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, help_title_, help_content_, &con_);

	// show info
	if (settings_.info)
		mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, rect, info_title_, info_content_, &con_);

	// show profiler
	if (settings_.profiler)
		profilerShow(rect);

	// show sensor
	if (settings_.sensor)
		sensorShow(smallrect);

	// finalize
	glfwSwapBuffers(window);
	render_mtx.unlock();
}

// Set window layout
void uiLayout(mjuiState *state)
{
	mjrRect *rect = state->rect;

	// set number of rectangles
	state->nrect = 4;

	// rect 0: entire framebuffer
	rect[0].left   = 0;
	rect[0].bottom = 0;
	glfwGetFramebufferSize(window_, &rect[0].width, &rect[0].height);

	// rect 1: UI 0
	rect[1].left   = 0;
	rect[1].width  = settings_.ui0 ? ui0_.width : 0;
	rect[1].bottom = 0;
	rect[1].height = rect[0].height;

	// rect 2: UI 1
	rect[2].width  = settings_.ui1 ? ui1_.width : 0;
	rect[2].left   = mjMAX(0, rect[0].width - rect[2].width);
	rect[2].bottom = 0;
	rect[2].height = rect[0].height;

	// rect 3: 3D plot (everything else is an overlay)
	rect[3].left   = rect[1].width;
	rect[3].width  = mjMAX(0, rect[0].width - rect[1].width - rect[2].width);
	rect[3].bottom = 0;
	rect[3].height = rect[0].height;
}

// Handle UI event
void uiEvent(mjuiState *state)
{
	int i;
	char err[200];

	// call UI 0 if event is directed to it
	if ((state->dragrect == ui0_.rectid) || (state->dragrect == 0 && state->mouserect == ui0_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// process UI event
		mjuiItem *it = mjui_event(&ui0_, state, &con_);

		// file section
		if (it && it->sectionid == SECT_FILE) {
			switch (it->itemid) {
				case 0: // save xml
					if (!mj_saveLastXML("mjmodel.xml", main_env_->model.get(), err, 200))
						ROS_ERROR("Save XML error: %s", err);
					break;

				case 1: // Save mjb
					mj_saveModel(main_env_->model.get(), "mjmodel.mjb", NULL, 0);
					break;

				case 2: // print model
					mj_printModel(main_env_->model.get(), "MJMODEL.TXT");
					break;

				case 3: // print data
					mj_printData(main_env_->model.get(), main_env_->data.get(), "MJDATA.TXT");
					break;

				case 4: // Quit
					settings_.exitrequest = 1;
					break;
			}
		}

		// option section
		else if (it && it->sectionid == SECT_OPTION) {
			switch (it->itemid) {
				case 0: // Spacing
					ui0_.spacing = mjui_themeSpacing(settings_.spacing);
					ui1_.spacing = mjui_themeSpacing(settings_.spacing);
					break;

				case 1: // Color
					ui0_.color = mjui_themeColor(settings_.color);
					ui1_.color = mjui_themeColor(settings_.color);
					break;

				case 2: // Font
					mjr_changeFont(50 * (settings_.font + 1), &con_);
					break;

				case 9: // Full screen
					if (glfwGetWindowMonitor(window_)) {
						// restore window from saved data
						glfwSetWindowMonitor(window_, NULL, windowpos_[0], windowpos_[1], windowsize_[0], windowsize_[1], 0);
					} else { // currently windowed: switch to fullscreen
						// save window data
						glfwGetWindowPos(window_, windowpos_, windowpos_ + 1);
						glfwGetWindowSize(window_, windowsize_, windowsize_ + 1);

						// switch
						glfwSetWindowMonitor(window_, glfwGetPrimaryMonitor(), 0, 0, vmode_.width, vmode_.height,
						                     vmode_.refreshRate);
					}

					// reinstate vsync, just in case
					glfwSwapInterval(settings_.vsync);
					break;

				case 10: // Vertical sync
					glfwSwapInterval(settings_.vsync);
					break;
			}

			// modify UI
			uiModify(window_, &ui0_, state, &con_);
			uiModify(window_, &ui1_, state, &con_);
		}

		// simulation section
		else if (it && it->sectionid == SECT_SIMULATION) {
			switch (it->itemid) {
				case 1: // reset
					resetSim();
					break;

				case 2: // Reload
					settings_.loadrequest = 1;
					break;

				case 3: // Align
					alignScale(main_env_->model);
					updateSettings(main_env_->model);
					break;

				case 4: // Copy Pose
					copyKey(main_env_->model, main_env_->data);
					break;

				case 5: // Adjust key
				case 6: // Reset to key
					// REVIEW: fully disable resetting to keypoints?
					i = settings_.key;
					// d_->time = m_->key_time[i]; // ros does not expect jumps back in time
					mju_copy(main_env_->data->qpos, main_env_->model->key_qpos + i * main_env_->model->nq,
					         main_env_->model->nq);
					mju_copy(main_env_->data->qvel, main_env_->model->key_qvel + i * main_env_->model->nv,
					         main_env_->model->nv);
					mju_copy(main_env_->data->act, main_env_->model->key_act + i * main_env_->model->na,
					         main_env_->model->na);
					mju_copy(main_env_->data->mocap_pos, main_env_->model->key_mpos + i * 3 * main_env_->model->nmocap,
					         3 * main_env_->model->nmocap);
					mju_copy(main_env_->data->mocap_quat, main_env_->model->key_mquat + i * 4 * main_env_->model->nmocap,
					         4 * main_env_->model->nmocap);
					mj_forward(main_env_->model.get(), main_env_->data.get());
					publishSimTime(main_env_->data->time);
					profilerUpdate(main_env_->model, main_env_->data);
					sensorUpdate(main_env_->model, main_env_->data);
					updateSettings(main_env_->model);
					break;

				case 7: // Set key
					i                             = settings_.key;
					main_env_->model->key_time[i] = main_env_->data->time;
					mju_copy(main_env_->model->key_qpos + i * main_env_->model->nq, main_env_->data->qpos,
					         main_env_->model->nq);
					mju_copy(main_env_->model->key_qvel + i * main_env_->model->nv, main_env_->data->qvel,
					         main_env_->model->nv);
					mju_copy(main_env_->model->key_act + i * main_env_->model->na, main_env_->data->act,
					         main_env_->model->na);
					mju_copy(main_env_->model->key_mpos + i * 3 * main_env_->model->nmocap, main_env_->data->mocap_pos,
					         3 * main_env_->model->nmocap);
					mju_copy(main_env_->model->key_mquat + i * 4 * main_env_->model->nmocap, main_env_->data->mocap_quat,
					         4 * main_env_->model->nmocap);
					break;
			}
		}

		// Physics section
		else if (it && it->sectionid == SECT_PHYSICS) {
			// Update disable flags in mjOption
			main_env_->model->opt.disableflags = 0;
			for (i = 0; i < mjNDISABLE; i++) {
				if (settings_.disable[i]) {
					main_env_->model->opt.disableflags |= (1 << i);
				}
			}

			// Update enable flags in mjOption
			main_env_->model->opt.enableflags = 0;
			for (i = 0; i < mjNENABLE; i++) {
				if (settings_.enable[i]) {
					main_env_->model->opt.enableflags |= (1 << i);
				}
			}
		}

		// Rendering section
		else if (it && it->sectionid == SECT_RENDERING) {
			// Set camera in mjvCamera
			if (settings_.camera == 0) {
				cam_.type = mjCAMERA_FREE;
			} else if (settings_.camera == 1) {
				if (pert_.select > 0) {
					cam_.type        = mjCAMERA_TRACKING;
					cam_.trackbodyid = pert_.select;
					cam_.fixedcamid  = -1;
				} else {
					cam_.type        = mjCAMERA_FREE;
					settings_.camera = 0;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &con_);
				}
			} else {
				cam_.type       = mjCAMERA_FIXED;
				cam_.fixedcamid = settings_.camera - 2;
			}
			// Print floating camera as MJCF element
			if (it->itemid == 3) {
				printCamera(scn_.camera);
			}
		}

		// Group section
		else if (it && it->sectionid == SECT_GROUP) {
			// Remake joint section if joint group changed
			if (it->name[0] == 'J' && it->name[1] == 'o') {
				ui1_.nsect = SECT_JOINT;
				makeJoint(main_env_->model, main_env_->data, ui1_.sect[SECT_JOINT].state);
				ui1_.nsect = NSECT1;
				uiModify(window_, &ui1_, state, &con_);
			}

			// Remake control section if actuator group changed
			if (it->name[0] == 'A' && it->name[1] == 'c') {
				ui1_.nsect = SECT_CONTROL;
				makeControl(main_env_->model, main_env_->data, ui1_.sect[SECT_CONTROL].state);
				ui1_.nsect = NSECT1;
				uiModify(window_, &ui1_, state, &con_);
			}
		}

		// Stop if UI processed event
		if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Call UI 1 if event is directed to it
	if ((state->dragrect == ui1_.rectid) || (state->dragrect == 0 && state->mouserect == ui1_.rectid) ||
	    state->type == mjEVENT_KEY) {
		// Process UI event
		mjuiItem *it = mjui_event(&ui1_, state, &con_);

		// control section
		if (it && it->sectionid == SECT_CONTROL) {
			// clear controls
			if (it->itemid == 0) {
				mju_zero(main_env_->data->ctrl, main_env_->model->nu);
				mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &con_);
			}
		}

		// Stop if UI processed event
		if (it != NULL || (state->type == mjEVENT_KEY && state->key == 0)) {
			return;
		}
	}

	// Short not handled by UI
	if (state->type == mjEVENT_KEY && state->key != 0) {
		switch (state->key) {
			case ' ': // Mode
				if (main_env_->model) {
					// if (sim_mode_ == simMode::PARALLEL) {
					// 	ROS_WARN_NAMED("mujoco", "Unpausing in PARALLEL sim mode is not allowed, use stepping instead!");
					// 	break;
					// }
					settings_.run = 1 - settings_.run;
					if (settings_.run)
						settings_.manual_env_steps = 0;
					pert_.active = 0;
					mjui_update(-1, -1, &ui0_, state, &con_);
				}
				break;

			case mjKEY_RIGHT: // Step forward
				if (main_env_->model && !settings_.run) {
					clearTimers(main_env_->data);
					settings_.manual_env_steps = 1;
				}
				break;

			case mjKEY_LEFT: // Step back
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_DOWN: // Step forward 100
				if (main_env_->model && !settings_.run) {
					clearTimers(main_env_->data);
					settings_.manual_env_steps = 100;
				}
				break;

			case mjKEY_UP: // Step backward 100
				ROS_DEBUG_THROTTLE_NAMED(1, "mujoco",
				                         "Stepping backwards is disabled as rostime should never run backwards.");
				break;

			case mjKEY_PAGE_UP: // Select parent body
				if (main_env_->model && pert_.select > 0) {
					pert_.select     = main_env_->model->body_parentid[pert_.select];
					pert_.skinselect = -1;

					// Stop perturbation if world reached
					if (pert_.select <= 0) {
						pert_.active = 0;
					}
				}
				break;

			case '-': //  Slow down
				if (settings_.slow_down < max_slow_down_ && !state->shift) {
					settings_.slow_down *= 2;
					settings_.speed_changed = true;
				}
				break;

			case '=': // Speed up
				if (settings_.slow_down > 1 && !state->shift) {
					settings_.slow_down /= 2;
					settings_.speed_changed = true;
				}
				break;
		}

		return;
	}

	// 3D Scroll
	if (state->type == mjEVENT_SCROLL && state->mouserect == 3 && main_env_->model) {
		// Emulate vertical mouse motion = 5% of window height
		mjv_moveCamera(main_env_->model.get(), mjMOUSE_ZOOM, 0, -0.05 * state->sy, &scn_, &cam_);

		return;
	}

	// 3D press
	if (state->type == mjEVENT_PRESS && state->mouserect == 3 && main_env_->model) {
		// Set perturbation
		int newperturb = 0;
		if (state->control && pert_.select > 0) {
			// right: translate; left: rotate
			if (state->right) {
				newperturb = mjPERT_TRANSLATE;
			} else if (state->left) {
				newperturb = mjPERT_ROTATE;
			}

			// Perturbation onset: reset reference
			if (newperturb && !pert_.active) {
				mjv_initPerturb(main_env_->model.get(), main_env_->data.get(), &scn_, &pert_);
			}
		}
		pert_.active = newperturb;

		// Handle double-click
		if (state->doubleclick) {
			// Determine selection mode
			int selmode;
			if (state->button == mjBUTTON_LEFT) {
				selmode = 1;
			} else if (state->control) {
				selmode = 3;
			} else {
				selmode = 2;
			}

			// Find geom and 3D click point, get corresponding body
			mjrRect r = state->rect[3];
			mjtNum selpnt[3];
			int selgeom, selskin;
			int selbody = mjv_select(main_env_->model.get(), main_env_->data.get(), &vopt_,
			                         (mjtNum)r.width / (mjtNum)r.height, (mjtNum)(state->x - r.left) / (mjtNum)r.width,
			                         (mjtNum)(state->y - r.bottom) / (mjtNum)r.height, &scn_, selpnt, &selgeom, &selskin);

			// Set lookat point, start tracking is requested
			if (selmode == 2 || selmode == 3) {
				// Copy selpnt if anything clicked
				if (selbody >= 0) {
					mju_copy3(cam_.lookat, selpnt);
				}

				// Switch to tracking camera if dynamic body clicked
				if (selmode == 3 && selbody > 0) {
					// Mujoco camera
					cam_.type        = mjCAMERA_TRACKING;
					cam_.trackbodyid = selbody;
					cam_.fixedcamid  = -1;

					// UI camera
					settings_.camera = 1;
					mjui_update(SECT_RENDERING, -1, &ui0_, &uistate_, &con_);
				}
			}

			// Set body selection
			else {
				if (selbody >= 0) {
					// Record selection
					pert_.select     = selbody;
					pert_.skinselect = selskin;

					// Compute localpos
					mjtNum tmp[3];
					mju_sub3(tmp, selpnt, main_env_->data->xpos + 3 * pert_.select);
					mju_mulMatTVec(pert_.localpos, main_env_->data->xmat + 9 * pert_.select, tmp, 3, 3);
				} else {
					pert_.select     = 0;
					pert_.skinselect = 0;
				}
			}

			// Stop perturbation on select
			pert_.active = 0;
		}

		return;
	}

	// 3D release
	if (state->type == mjEVENT_RELEASE && state->dragrect == 3 && main_env_->model) {
		// Stop perturbation
		pert_.active = 0;

		return;
	}

	// 3D move
	if (state->type == mjEVENT_MOVE && state->dragrect == 3 && main_env_->model) {
		// Determine action base on mouse button
		mjtMouse action;
		if (state->right) {
			action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
		} else if (state->left) {
			action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
		} else {
			action = mjMOUSE_ZOOM;
		}

		// Move perturb or camera
		mjrRect r = state->rect[3];
		if (pert_.active) {
			mjv_movePerturb(main_env_->model.get(), main_env_->data.get(), action, state->dx / r.height,
			                -state->dy / r.height, &scn_, &pert_);
		} else {
			mjv_moveCamera(main_env_->model.get(), action, state->dx / r.height, -state->dy / r.height, &scn_, &cam_);
		}

		return;
	}
}

// print current camera as MJCF specification
void printCamera(mjvGLCamera *camera)
{
	mjtNum cam_right[3];
	mjtNum cam_forward[3];
	mjtNum cam_up[3];
	mju_f2n(cam_forward, camera[0].forward, 3);
	mju_f2n(cam_up, camera[0].up, 3);
	mju_cross(cam_right, cam_forward, cam_up);
	std::printf("<camera pos=\"%.3f %.3f %.3f\" xyaxes=\"%.3f %.3f %.3f %.3f %.3f %.3f\"/>\n",
	            (camera[0].pos[0] + camera[1].pos[0]) / 2, (camera[0].pos[1] + camera[1].pos[1]) / 2,
	            (camera[0].pos[2] + camera[1].pos[2]) / 2, cam_right[0], cam_right[1], cam_right[2], camera[0].up[0],
	            camera[0].up[1], camera[0].up[2]);
}

// copy qpos to clipboard as key
void copyKey(mjModelPtr model, mjDataPtr data)
{
	char clipboard[5000] = "<key qpos='";
	char buf[200];

	// prepare string
	for (int i = 0; i < model->nq; i++) {
		mju::sprintf_arr(buf, i == model->nq - 1 ? "%g" : "%g ", data->qpos[i]);
		mju::strcat_arr(clipboard, buf);
	}
	mju::strcat_arr(clipboard, "'/>");

	// copy to clipboard
	glfwSetClipboardString(window_, clipboard);
}

void sensorInit(void)
{
	// set figure to default
	mjv_defaultFigure(&figsensor_);
	figsensor_.figurergba[3] = 0.5f;

	// set flags
	figsensor_.flg_extend    = 1;
	figsensor_.flg_barplot   = 1;
	figsensor_.flg_symmetric = 1;

	// title
	mju::strcpy_arr(figsensor_.title, "Sensor data");

	// y-tick number format
	mju::strcpy_arr(figsensor_.yformat, "%0f");

	// grid size
	figsensor_.gridsize[0] = 2;
	figsensor_.gridsize[1] = 3;

	// minimum range
	figsensor_.range[0][0] = 0;
	figsensor_.range[0][0] = 0;
	figsensor_.range[1][0] = -1;
	figsensor_.range[1][1] = 1;
}

// Update sensor figure
void sensorUpdate(mjModelPtr model, mjDataPtr data)
{
	static const int maxline = 10;

	// clear linepnt
	for (int i = 0; i < maxline; i++) {
		figsensor_.linepnt[i] = 0;
	}

	// start with line 0
	int lineid = 0;

	// loop over sensors
	for (int n = 0; n < model->nsensor; n++) {
		// go to next line if type is different
		if (n > 0 && model->sensor_type[n] != model->sensor_type[n - 1])
			lineid = mjMIN(lineid + 1, maxline - 1);

		// get info about this sensor
		mjtNum cutoff = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
		int adr       = model->sensor_adr[n];
		int dim       = model->sensor_dim[n];

		// data pointer in line
		int p = figsensor_.linepnt[lineid];

		// fill in data for this sensor
		for (int i = 0; i < dim; i++) {
			// check size
			if ((p + 2 * i) > mjMAXLINEPNT / 2)
				break;

			// x
			figsensor_.linedata[lineid][2 * p + 4 * i]     = (float)(adr + i);
			figsensor_.linedata[lineid][2 * p + 4 * i + 2] = (float)(adr + i);

			// y
			figsensor_.linedata[lineid][2 * p + 4 * i + 1] = 0;
			figsensor_.linedata[lineid][2 * p + 4 * i + 3] = (float)(data->sensordata[adr + i] / cutoff);
		}

		// update linepnt
		figsensor_.linepnt[lineid] = mjMIN(mjMAXLINEPNT - 1, figsensor_.linepnt[lineid] + 2 * dim);
	}
}

// Show sensor figure
void sensorShow(mjrRect rect)
{
	// constant width with and without profiler
	int width = settings_.profiler ? rect.width / 3 : rect.width / 4;

	// render figure on the right
	mjrRect viewport = { rect.left + rect.width - width, rect.bottom, width, rect.height / 3 };
	mjr_figure(viewport, &figsensor_, &con_);
}

void profilerInit(void)
{
	int i, n;

	// set figures to default
	mjv_defaultFigure(&figconstraint_);
	mjv_defaultFigure(&figcost_);
	mjv_defaultFigure(&figtimer_);
	mjv_defaultFigure(&figsize_);

	// titles
	mju::strcpy_arr(figconstraint_.title, "Counts");
	mju::strcpy_arr(figcost_.title, "Convergence (log 10)");
	mju::strcpy_arr(figsize_.title, "Dimensions");
	mju::strcpy_arr(figtimer_.title, "CPU time (msec)");

	// x-labels
	mju::strcpy_arr(figconstraint_.xlabel, "Solver iteration");
	mju::strcpy_arr(figcost_.xlabel, "Solver iteration");
	mju::strcpy_arr(figsize_.xlabel, "Video Frame");
	mju::strcpy_arr(figtimer_.xlabel, "Video Frame");

	// y-tick number formats
	mju::strcpy_arr(figconstraint_.yformat, "%.0f");
	mju::strcpy_arr(figcost_.yformat, "%.1f");
	mju::strcpy_arr(figsize_.yformat, "%.0f");
	mju::strcpy_arr(figtimer_.yformat, "%.2f");

	// colors
	figconstraint_.figurergba[0] = 0.1f;
	figcost_.figurergba[2]       = 0.2f;
	figsize_.figurergba[0]       = 0.1f;
	figtimer_.figurergba[2]      = 0.2f;
	figconstraint_.figurergba[3] = 0.5f;
	figcost_.figurergba[3]       = 0.5f;
	figsize_.figurergba[3]       = 0.5f;
	figtimer_.figurergba[3]      = 0.5f;

	// legends
	mju::strcpy_arr(figconstraint_.linename[0], "total");
	mju::strcpy_arr(figconstraint_.linename[1], "active");
	mju::strcpy_arr(figconstraint_.linename[2], "changed");
	mju::strcpy_arr(figconstraint_.linename[3], "evals");
	mju::strcpy_arr(figconstraint_.linename[4], "updates");
	mju::strcpy_arr(figcost_.linename[0], "improvement");
	mju::strcpy_arr(figcost_.linename[1], "gradient");
	mju::strcpy_arr(figcost_.linename[2], "lineslope");
	mju::strcpy_arr(figsize_.linename[0], "dof");
	mju::strcpy_arr(figsize_.linename[1], "body");
	mju::strcpy_arr(figsize_.linename[2], "constraint");
	mju::strcpy_arr(figsize_.linename[3], "sqrt(nnz)");
	mju::strcpy_arr(figsize_.linename[4], "contact");
	mju::strcpy_arr(figsize_.linename[5], "iteration");
	mju::strcpy_arr(figtimer_.linename[0], "total");
	mju::strcpy_arr(figtimer_.linename[1], "collision");
	mju::strcpy_arr(figtimer_.linename[2], "prepare");
	mju::strcpy_arr(figtimer_.linename[3], "solve");
	mju::strcpy_arr(figtimer_.linename[4], "other");

	// grid sizes
	figconstraint_.gridsize[0] = 5;
	figconstraint_.gridsize[1] = 5;
	figcost_.gridsize[0]       = 5;
	figcost_.gridsize[1]       = 5;
	figsize_.gridsize[0]       = 3;
	figsize_.gridsize[1]       = 5;
	figtimer_.gridsize[0]      = 3;
	figtimer_.gridsize[1]      = 5;

	// minimum ranges
	figconstraint_.range[0][0] = 0;
	figconstraint_.range[0][1] = 20;
	figconstraint_.range[1][0] = 0;
	figconstraint_.range[1][1] = 80;
	figcost_.range[0][0]       = 0;
	figcost_.range[0][1]       = 20;
	figcost_.range[1][0]       = -15;
	figcost_.range[1][1]       = 5;
	figsize_.range[0][0]       = -200;
	figsize_.range[0][1]       = 0;
	figsize_.range[1][0]       = 0;
	figsize_.range[1][1]       = 100;
	figtimer_.range[0][0]      = -200;
	figtimer_.range[0][1]      = 0;
	figtimer_.range[1][0]      = 0;
	figtimer_.range[1][1]      = 0.4f;

	// init x axis on history figures (do not show yet)
	for (n = 0; n < 6; n++)
		for (i = 0; i < mjMAXLINEPNT; i++) {
			figtimer_.linedata[n][2 * i] = (float)-i;
			figsize_.linedata[n][2 * i]  = (float)-i;
		}
}

void profilerUpdate(mjModelPtr model, mjDataPtr data)
{
	int i, n;

	// update constraint figure
	figconstraint_.linepnt[0] = mjMIN(mjMIN(data->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 5; i++) {
		figconstraint_.linepnt[i] = figconstraint_.linepnt[0];
	}
	if (model->opt.solver == mjSOL_PGS) {
		figconstraint_.linepnt[3] = 0;
		figconstraint_.linepnt[4] = 0;
	}
	if (model->opt.solver == mjSOL_CG) {
		figconstraint_.linepnt[4] = 0;
	}
	for (i = 0; i < figconstraint_.linepnt[0]; i++) {
		// x
		figconstraint_.linedata[0][2 * i] = (float)i;
		figconstraint_.linedata[1][2 * i] = (float)i;
		figconstraint_.linedata[2][2 * i] = (float)i;
		figconstraint_.linedata[3][2 * i] = (float)i;
		figconstraint_.linedata[4][2 * i] = (float)i;

		// y
		figconstraint_.linedata[0][2 * i + 1] = (float)data->nefc;
		figconstraint_.linedata[1][2 * i + 1] = (float)data->solver[i].nactive;
		figconstraint_.linedata[2][2 * i + 1] = (float)data->solver[i].nchange;
		figconstraint_.linedata[3][2 * i + 1] = (float)data->solver[i].neval;
		figconstraint_.linedata[4][2 * i + 1] = (float)data->solver[i].nupdate;
	}

	// update cost figure
	figcost_.linepnt[0] = mjMIN(mjMIN(data->solver_iter, mjNSOLVER), mjMAXLINEPNT);
	for (i = 1; i < 3; i++) {
		figcost_.linepnt[i] = figcost_.linepnt[0];
	}
	if (model->opt.solver == mjSOL_PGS) {
		figcost_.linepnt[1] = 0;
		figcost_.linepnt[2] = 0;
	}

	for (i = 0; i < figcost_.linepnt[0]; i++) {
		// x
		figcost_.linedata[0][2 * i] = (float)i;
		figcost_.linedata[1][2 * i] = (float)i;
		figcost_.linedata[2][2 * i] = (float)i;

		// y
		figcost_.linedata[0][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, data->solver[i].improvement));
		figcost_.linedata[1][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, data->solver[i].gradient));
		figcost_.linedata[2][2 * i + 1] = (float)mju_log10(mju_max(mjMINVAL, data->solver[i].lineslope));
	}

	// get timers: total, collision, prepare, solve, other
	mjtNum total = data->timer[mjTIMER_STEP].duration;
	int number   = data->timer[mjTIMER_STEP].number;
	if (!number) {
		total  = data->timer[mjTIMER_FORWARD].duration;
		number = data->timer[mjTIMER_FORWARD].number;
	}
	number         = mjMAX(1, number);
	float tdata[5] = { (float)(total / number), (float)(data->timer[mjTIMER_POS_COLLISION].duration / number),
		                (float)(data->timer[mjTIMER_POS_MAKE].duration / number) +
		                    (float)(data->timer[mjTIMER_POS_PROJECT].duration / number),
		                (float)(data->timer[mjTIMER_CONSTRAINT].duration / number), 0 };
	tdata[4]       = tdata[0] - tdata[1] - tdata[2] - tdata[3];

	// update figtimer
	int pnt = mjMIN(201, figtimer_.linepnt[0] + 1);
	for (n = 0; n < 5; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figtimer_.linedata[n][2 * i + 1] = figtimer_.linedata[n][2 * i - 1];
		}

		// assign new
		figtimer_.linepnt[n]     = pnt;
		figtimer_.linedata[n][1] = tdata[n];
	}

	// get sizes: nv, nbody, nefc, sqrt(nnz), ncont, iter
	float sdata[6] = { (float)model->nv,  (float)model->nbody,
		                (float)data->nefc, (float)mju_sqrt((mjtNum)data->solver_nnz),
		                (float)data->ncon, (float)data->solver_iter };

	// update figsize
	pnt = mjMIN(201, figsize_.linepnt[0] + 1);
	for (n = 0; n < 6; n++) {
		// shift data
		for (i = pnt - 1; i > 0; i--) {
			figsize_.linedata[n][2 * i + 1] = figsize_.linedata[n][2 * i - 1];
		}

		// assign new
		figsize_.linepnt[n]     = pnt;
		figsize_.linedata[n][1] = sdata[n];
	}
}

void profilerShow(mjrRect rect)
{
	mjrRect viewport = { rect.left + rect.width - rect.width / 4, rect.bottom, rect.width / 4, rect.height / 4 };
	mjr_figure(viewport, &figtimer_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figsize_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figcost_, &con_);
	viewport.bottom += rect.height / 4;
	mjr_figure(viewport, &figconstraint_, &con_);
}

// Update UI 0 when MuJoCo structures change (except for joint sliders)
void updateSettings(mjModelPtr model)
{
	int i;

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating physics");
	// physics flags
	for (i = 0; i < mjNDISABLE; i++) {
		settings_.disable[i] = ((model->opt.disableflags & (i << i)) != 0);
	}
	for (i = 0; i < mjNENABLE; i++) {
		settings_.enable[i] = ((model->opt.enableflags & (1 << 1)) != 0);
	}

	ROS_DEBUG_ONCE_NAMED("mujoco", "\tupdating cam");
	// camera
	if (cam_.type == mjCAMERA_FIXED) {
		settings_.camera = 2 + cam_.fixedcamid;
	} else if (cam_.type == mjCAMERA_TRACKING) {
		settings_.camera = 1;
	} else {
		settings_.camera = 0;
	}

	// update UI
	if (vis_)
		mjui_update(-1, -1, &ui0_, &uistate_, &con_);
}

// Physics section of UI
void makePhysics(mjModelPtr model, int oldstate)
{
	int i;

	mjuiDef defPhysics[]     = { { mjITEM_SECTION, "Physics", oldstate, nullptr, "AP" },
                            { mjITEM_SELECT, "Integrator", 2, &(model->opt.integrator), "Euler\nRK4" },
                            { mjITEM_SELECT, "Collision", 2, &(model->opt.collision), "All\nPair\nDynamic" },
                            { mjITEM_SELECT, "Cone", 2, &(model->opt.cone), "Pyramidal\nElliptic" },
                            { mjITEM_SELECT, "Jacobian", 2, &(model->opt.jacobian), "Dense\nSparse\nAuto" },
                            { mjITEM_SELECT, "Solver", 2, &(model->opt.solver), "PGS\nCG\nNewton" },
                            { mjITEM_SEPARATOR, "Algorithmic Parameters", 1 },
                            { mjITEM_EDITNUM, "Timestep", 2, &(model->opt.timestep), "1 0 1" },
                            { mjITEM_EDITINT, "Iterations", 2, &(model->opt.iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Tolerance", 2, &(model->opt.tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "Noslip Iter", 2, &(model->opt.noslip_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "Noslip Tol", 2, &(model->opt.noslip_tolerance), "1 0 1" },
                            { mjITEM_EDITINT, "MPR Iter", 2, &(model->opt.mpr_iterations), "1 0 1000" },
                            { mjITEM_EDITNUM, "MPR Tol", 2, &(model->opt.mpr_tolerance), "1 0 1" },
                            { mjITEM_EDITNUM, "API Rate", 2, &(model->opt.apirate), "1 0 1000" },
                            { mjITEM_SEPARATOR, "Physical Parameters", 1 },
                            { mjITEM_EDITNUM, "Gravity", 2, model->opt.gravity, "3" },
                            { mjITEM_EDITNUM, "Wind", 2, model->opt.wind, "3" },
                            { mjITEM_EDITNUM, "Magnetic", 2, model->opt.magnetic, "3" },
                            { mjITEM_EDITNUM, "Density", 2, &(model->opt.density), "1" },
                            { mjITEM_EDITNUM, "Viscosity", 2, &(model->opt.viscosity), "1" },
                            { mjITEM_EDITNUM, "Imp Ratio", 2, &(model->opt.impratio), "1" },
                            { mjITEM_SEPARATOR, "Disable Flags", 1 },
                            { mjITEM_END } };
	mjuiDef defEnableFlags[] = { { mjITEM_SEPARATOR, "Enable Flags", 1 }, { mjITEM_END } };
	mjuiDef defOverride[]    = { { mjITEM_SEPARATOR, "Contact Override", 1 },
                             { mjITEM_EDITNUM, "Margin", 2, &(model->opt.o_margin), "1" },
                             { mjITEM_EDITNUM, "Sol Imp", 2, &(model->opt.o_solimp), "5" },
                             { mjITEM_EDITNUM, "Sol Ref", 2, &(model->opt.o_solref), "2" },
                             { mjITEM_END } };

	// add physics
	mjui_add(&ui0_, defPhysics);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKINT, "", 2, nullptr, "" }, { mjITEM_END } };
	for (i = 0; i < mjNDISABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjDISABLESTRING[i]);
		defFlag[0].pdata = settings_.disable + i;
		mjui_add(&ui0_, defFlag);
	}
	mjui_add(&ui0_, defEnableFlags);
	for (i = 0; i < mjNENABLE; i++) {
		mju::strcpy_arr(defFlag[0].name, mjENABLESTRING[i]);
		defFlag[0].pdata = settings_.enable + i;
		mjui_add(&ui0_, defFlag);
	}

	// add contact override
	mjui_add(&ui0_, defOverride);
}

// Make rendering section of UI
void makeRendering(mjModelPtr model, int oldstate)
{
	int i, j;

	mjuiDef defRendering[] = {
		{ mjITEM_SECTION, "Rendering", oldstate, nullptr, "AR" },
		{ mjITEM_SELECT, "Camera", 2, &(settings_.camera), "Free\nTracking" },
		{ mjITEM_SELECT, "Label", 2, &(vopt_.label),
		  "None\nBody\nJoint\nGeom\nSite\nCamera\nLight\nTendon\nActuator\nConstraint\nSkin\nSelection\nSel Pnt\nForce" },
		{ mjITEM_SELECT, "Frame", 2, &(vopt_.frame), "None\nBody\nGeom\nSite\nCamera\nLight\nWorld" },
		{ mjITEM_BUTTON, "Copy camera", 2, nullptr, "" },
		{
		    mjITEM_SEPARATOR,
		    "Model Elements",
		    1,
		},
		{ mjITEM_END }
	};
	mjuiDef defOpenGL[] = { { mjITEM_SEPARATOR, "OpenGL Effects", 1 }, { mjITEM_END } };

	// add model cameras, up to UI limit
	for (i = 0; i < mjMIN(model->ncam, mjMAXUIMULTI - 2); i++) {
		// prepare name
		char camname[mjMAXUITEXT] = "\n";
		if (model->names[model->name_camadr[i]]) {
			mju::strcat_arr(camname, model->names + model->name_camadr[i]);
		} else {
			mju::sprintf_arr(camname, "\nCamera %d", i);
		}

		// check string length
		if (mju::strlen_arr(camname) + mju::strlen_arr(defRendering[1].other) >= mjMAXUITEXT - 1) {
			break;
		}

		// add camera
		mju::strcat_arr(defRendering[1].other, camname);
	}

	// add rendering standard
	mjui_add(&ui0_, defRendering);

	// add flags programmatically
	mjuiDef defFlag[] = { { mjITEM_CHECKBYTE, "", 2, nullptr, "" }, { mjITEM_END } };
	for (i = 0; i < mjNVISFLAG; i++) {
		// set name, remove "&"
		mju::strcpy_arr(defFlag[0].name, mjVISSTRING[i][0]);
		for (j = 0; j < strlen(mjVISSTRING[i][0]); j++) {
			if (mjVISSTRING[i][0][j] == '&') {
				mju_strncpy(defFlag[0].name + j, mjVISSTRING[i][0] + j + 1, mju::sizeof_arr(defFlag[0].name) - j);
				break;
			}
		}

		// set shortcut and data
		mju::sprintf_arr(defFlag[0].other, " %s", mjVISSTRING[i][2]);
		defFlag[0].pdata = vopt_.flags + i;
		mjui_add(&ui0_, defFlag);
	}
	mjui_add(&ui0_, defOpenGL);
	for (i = 0; i < mjNRNDFLAG; i++) {
		mju::strcpy_arr(defFlag[0].name, mjRNDSTRING[i][0]);
		if (mjRNDSTRING[i][2][0])
			mju::sprintf_arr(defFlag[0].other, " %s", mjRNDSTRING[i][2]);
		defFlag[0].pdata = scn_.flags + i;
		mjui_add(&ui0_, defFlag);
	}
}

// Make group section UI
void makeGroup(int oldstate)
{
	mjuiDef defGroup[] = { { mjITEM_SECTION, "Group enable", oldstate, nullptr, "AG" },
		                    { mjITEM_SEPARATOR, "Geom groups", 1 },
		                    { mjITEM_CHECKBYTE, "Geom 0", 2, vopt_.geomgroup, " 0" },
		                    { mjITEM_CHECKBYTE, "Geom 1", 2, vopt_.geomgroup + 1, " 1" },
		                    { mjITEM_CHECKBYTE, "Geom 2", 2, vopt_.geomgroup + 2, " 2" },
		                    { mjITEM_CHECKBYTE, "Geom 3", 2, vopt_.geomgroup + 3, " 3" },
		                    { mjITEM_CHECKBYTE, "Geom 4", 2, vopt_.geomgroup + 4, " 4" },
		                    { mjITEM_CHECKBYTE, "Geom 5", 2, vopt_.geomgroup + 5, " 5" },
		                    { mjITEM_SEPARATOR, "Site groups", 1 },
		                    { mjITEM_CHECKBYTE, "Site 0", 2, vopt_.sitegroup, "S0" },
		                    { mjITEM_CHECKBYTE, "Site 1", 2, vopt_.sitegroup + 1, "S1" },
		                    { mjITEM_CHECKBYTE, "Site 2", 2, vopt_.sitegroup + 2, "S2" },
		                    { mjITEM_CHECKBYTE, "Site 3", 2, vopt_.sitegroup + 3, "S3" },
		                    { mjITEM_CHECKBYTE, "Site 4", 2, vopt_.sitegroup + 4, "S4" },
		                    { mjITEM_CHECKBYTE, "Site 5", 2, vopt_.sitegroup + 5, "S5" },
		                    { mjITEM_SEPARATOR, "Joint groups", 1 },
		                    { mjITEM_CHECKBYTE, "Joint 0", 2, vopt_.jointgroup, "" },
		                    { mjITEM_CHECKBYTE, "Joint 1", 2, vopt_.jointgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Joint 2", 2, vopt_.jointgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Joint 3", 2, vopt_.jointgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Joint 4", 2, vopt_.jointgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Joint 5", 2, vopt_.jointgroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Tendon groups", 1 },
		                    { mjITEM_CHECKBYTE, "Tendon 0", 2, vopt_.tendongroup, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 1", 2, vopt_.tendongroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 2", 2, vopt_.tendongroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 3", 2, vopt_.tendongroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 4", 2, vopt_.tendongroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Tendon 5", 2, vopt_.tendongroup + 5, "" },
		                    { mjITEM_SEPARATOR, "Actuator groups", 1 },
		                    { mjITEM_CHECKBYTE, "Actuator 0", 2, vopt_.actuatorgroup, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 1", 2, vopt_.actuatorgroup + 1, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 2", 2, vopt_.actuatorgroup + 2, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 3", 2, vopt_.actuatorgroup + 3, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 4", 2, vopt_.actuatorgroup + 4, "" },
		                    { mjITEM_CHECKBYTE, "Actuator 5", 2, vopt_.actuatorgroup + 5, "" },
		                    { mjITEM_END } };

	mjui_add(&ui0_, defGroup);
}

// Make joint section of UI
void makeJoint(mjModelPtr model, mjDataPtr data, int oldstate)
{
	int i;

	mjuiDef defJoint[] = {
		{ mjITEM_SECTION, "Joint", oldstate, nullptr, "AJ" },
		{ mjITEM_END },
	};
	mjuiDef defSlider[] = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// add section
	mjui_add(&ui1_, defJoint);
	defSlider[0].state = 4;

	// add scalar joints, exit if UI limit reached
	int itemcnt = 0;
	for (i = 0; i < model->njnt && itemcnt < mjMAXUIITEM; i++) {
		if ((model->jnt_type[i] == mjJNT_HINGE || model->jnt_type[i] == mjJNT_SLIDE)) {
			// skip if joint group is disabled
			if (!vopt_.jointgroup[mjMAX(0, mjMIN(mjNGROUP - 1, model->jnt_group[i]))]) {
				continue;
			}

			// set data and name
			defSlider[0].pdata = data->qpos + model->jnt_qposadr[i];
			if (model->names[model->name_jntadr[i]]) {
				mju::strcpy_arr(defSlider[0].name, model->names + model->name_jntadr[i]);
			} else {
				mju::sprintf_arr(defSlider[0].name, "joint %d", i);
			}

			// set range
			if (model->jnt_limited[i]) {
				mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", model->jnt_range[2 * i], model->jnt_range[2 * i + 1]);
			} else if (model->jnt_type[i] == mjJNT_SLIDE) {
				mju::strcpy_arr(defSlider[0].other, "-1 1");
			} else {
				mju::strcpy_arr(defSlider[0].other, "-3.1416 3.1416");
			}

			// add and count
			mjui_add(&ui1_, defSlider);
			itemcnt++;
		}
	}
}

// Make control section of UI
void makeControl(mjModelPtr model, mjDataPtr data, int oldstate)
{
	int i;

	mjuiDef defControl[] = { { mjITEM_SECTION, "Control", oldstate, nullptr, "AC" },
		                      { mjITEM_BUTTON, "Clear all", 2 },
		                      { mjITEM_END } };
	mjuiDef defSlider[]  = { { mjITEM_SLIDERNUM, "", 2, nullptr, "0 1" }, { mjITEM_END } };

	// Add section
	mjui_add(&ui1_, defControl);
	defSlider[0].state = 2;

	// Add controls, exit if UI limit reached (Clear button already added)
	int itemcnt = 1;
	for (i = 0; i < model->nu && itemcnt < mjMAXUIITEM; i++) {
		// Skip if actuator group is disabled
		if (!vopt_.actuatorgroup[mjMAX(0, mjMIN(mjNGROUP - 1, model->actuator_group[i]))]) {
			continue;
		}

		// set data and name
		defSlider[0].pdata = data->ctrl + i;
		if (model->names[model->name_actuatoradr[i]]) {
			mju::strcpy_arr(defSlider[0].name, model->names + model->name_actuatoradr[i]);
		} else {
			mju::sprintf_arr(defSlider[0].name, "control %d", i);
		}

		// set range
		if (model->actuator_ctrllimited[i]) {
			mju::sprintf_arr(defSlider[0].other, "%.4g %.4g", model->actuator_ctrlrange[2 * i + 1]);
		} else {
			mju::strcpy_arr(defSlider[0].other, "-1 1");
		}

		// add and count
		mjui_add(&ui1_, defSlider);
		itemcnt++;
	}
}

// Make model-dependent UI sections
void makeSections(void)
{
	int i;

	// get section open-close state, UI 0
	int oldstate0[NSECT0];
	for (i = 0; i < NSECT0; i++) {
		oldstate0[i] = 0;
		if (ui0_.nsect > i) {
			oldstate0[i] = ui0_.sect[i].state;
		}
	}

	// get section open-close state, UI 1
	int oldstate1[NSECT1];
	for (i = 0; i < NSECT1; i++) {
		oldstate1[i] = 0;
		if (ui1_.nsect > i) {
			oldstate1[i] = ui1_.sect[i].state;
		}
	}

	// clear model-dependent sections of UI
	ui0_.nsect = SECT_PHYSICS;
	ui1_.nsect = 0;

	// make
	makePhysics(main_env_->model, oldstate0[SECT_PHYSICS]);
	makeRendering(main_env_->model, oldstate0[SECT_RENDERING]);
	makeGroup(oldstate0[SECT_GROUP]);
	makeJoint(main_env_->model, main_env_->data, oldstate1[SECT_JOINT]);
	makeControl(main_env_->model, main_env_->data, oldstate1[SECT_CONTROL]);
}

// Prepare to render
void prepare(mjModelPtr model, mjDataPtr data)
{
	// data for FPS calculation
	static double lastupdatetm = 0;

	// update interval, save update time
	double tmnow    = glfwGetTime();
	double interval = tmnow - lastupdatetm;
	interval        = mjMIN(1, mjMAX(0.0001, interval));
	lastupdatetm    = tmnow;

	// No model: nothing to do
	if (!model) {
		return;
	}

	// Update scene
	mjv_updateScene(model.get(), data.get(), &vopt_, &pert_, &cam_, mjCAT_ALL, &scn_);

	// Update watch
	if (settings_.ui0 && ui0_.sect[SECT_WATCH].state) {
		watch(model, data);
		mjui_update(SECT_WATCH, -1, &ui0_, &uistate_, &con_);
	}

	// Update joint
	if (settings_.ui1 && ui1_.sect[SECT_JOINT].state) {
		mjui_update(SECT_JOINT, -1, &ui1_, &uistate_, &con_);
	}

	// Update info text
	if (settings_.info) {
		infotext(model, data, info_title_, info_content_, interval);
	}

	// Update control
	if (settings_.ui1 && ui1_.sect[SECT_CONTROL].state) {
		mjui_update(SECT_CONTROL, -1, &ui1_, &uistate_, &con_);
	}

	// Update profiler
	if (settings_.profiler && settings_.run) {
		profilerUpdate(model, data);
	}

	// Update sensor
	if (settings_.sensor && settings_.run) {
		sensorUpdate(model, data);
	}

	// clear timers once profiler info has been copied
	clearTimers(data);
}

// Sprintf forwarding, to avoid compiler warning in x-macro
void printField(char (&str)[mjMAXUINAME], void *ptr)
{
	mju::sprintf_arr(str, "%g", *(mjtNum *)ptr);
}

// Update watch
void watch(mjModelPtr model, mjDataPtr data)
{
	// clear
	ui0_.sect[SECT_WATCH].item[2].multi.nelem = 1;
	mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid field");

	// prepare symbols needed by xmacro
	MJDATA_POINTERS_PREAMBLE(model);

// find specified field in mjData arrays, update value
#define X(TYPE, NAME, NR, NC)                                                                   \
	if (!mju::strcmp_arr(#NAME, settings_.field) && !mju::strcmp_arr(#TYPE, "mjtNum")) {         \
		if (settings_.index >= 0 && settings_.index < model->NR * NC) {                           \
			printField(ui0_.sect[SECT_WATCH].item[2].multi.name[0], data->NAME + settings_.index); \
		} else {                                                                                  \
			mju::strcpy_arr(ui0_.sect[SECT_WATCH].item[2].multi.name[0], "invalid index");         \
		}                                                                                         \
		return;                                                                                   \
	}

	MJDATA_POINTERS
#undef X
}

// Prepare info text
void infotext(mjModelPtr model, mjDataPtr data, char (&title)[kBufSize], char (&content)[kBufSize], double interval)
{
	char tmp[20];

	// Compute solver error
	mjtNum solerr = 0;
	if (data->solver_iter) {
		int ind = mjMIN(data->solver_iter - 1, mjNSOLVER - 1);
		solerr  = mju_min(data->solver[ind].improvement, data->solver[ind].gradient);
		if (solerr == 0) {
			solerr = mju_max(data->solver[ind].improvement, data->solver[ind].gradient);
		}
	}
	solerr = mju_log10(mju_max(mjMINVAL, solerr));

	const std::string realtime_nominator = settings_.slow_down == 1 ? "" : "1/";
	mju::strcpy_arr(title, "Time\nSize\nCPU\nSolver\nFPS\nstack\nconbuf\nefcbuf");
	mju::sprintf_arr(content, "%-9.3f %s%d x\n%d  (%d con)\n%.3f\n%.1f  (%d it)\n%.0f\n%.3f\n%.3f\n%.3f", data->time,
	                 realtime_nominator.c_str(), settings_.slow_down, data->nefc, data->ncon,
	                 settings_.run ?
	                     data->timer[mjTIMER_STEP].duration / mjMAX(1, data->timer[mjTIMER_STEP].number) :
	                     data->timer[mjTIMER_FORWARD].duration / mjMAX(1, data->timer[mjTIMER_FORWARD].number),
	                 solerr, data->solver_iter, 1 / interval, data->maxuse_stack / (double)data->nstack,
	                 data->maxuse_con / (double)model->nconmax, data->maxuse_efc / (double)model->njmax);

	// Add energy if enabled
	if (mjENABLED_ros(model, mjENBL_ENERGY)) {
		mju::sprintf_arr(tmp, "\n%.3f", data->energy[0] + data->energy[1]);
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nEnergy");
	}

	// Add FwdInv if enabled
	if (mjENABLED_ros(model, mjENBL_FWDINV)) {
		mju::sprintf_arr(tmp, "\n%.1f %.1f", mju_log10(mju_max(mjMINVAL, data->solver_fwdinv[0])),
		                 mju_log10(mju_max(mjMINVAL, data->solver_fwdinv[1])));
		mju::strcat_arr(content, tmp);
		mju::strcat_arr(title, "\nFwdInv");
	}
}

//---------------------------------- utility functions --------------------------------------
// Align and scale view
void alignScale(mjModelPtr model)
{
	// autoscale
	cam_.lookat[0] = model->stat.center[0];
	cam_.lookat[1] = model->stat.center[1];
	cam_.lookat[2] = model->stat.center[2];
	cam_.distance  = 1.5 * model->stat.extent;

	// set to free camera
	cam_.type = mjCAMERA_FREE;
}

// Clear all timers
void clearTimers(mjDataPtr data)
{
	for (int i = 0; i < mjNTIMER; i++) {
		data->timer[i].duration = 0;
		data->timer[i].number   = 0;
	}
}

// millisecond timer, for MuJoCo built-in profiler
mjtNum timer(void)
{
	return (mjtNum)(1000 * glfwGetTime());
}

void setupCallbacks()
{
	if (sim_mode_ == simMode::SINGLE)
		service_servers_.push_back(nh_->advertiseService("set_pause", setPauseCB));
	service_servers_.push_back(nh_->advertiseService("shutdown", shutdownCB));
	service_servers_.push_back(nh_->advertiseService("reset", resetCB));
	service_servers_.push_back(nh_->advertiseService("set_model_state", setModelStateCB));
	service_servers_.push_back(nh_->advertiseService("get_model_state", getModelStateCB));
	service_servers_.push_back(nh_->advertiseService("set_geom_properties", setGeomPropertiesCB));
	service_servers_.push_back(nh_->advertiseService("reset_model_qpos", resetBodyQPosCB));

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
	settings_.run = !req.paused;
	if (settings_.run)
		settings_.manual_env_steps = 0;
	return true;
}

bool resetCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
	resetSim();
	return true;
}

void onStepGoal(const mujoco_ros_msgs::StepGoalConstPtr &goal)
{
	mujoco_ros_msgs::StepResult result;

	if (settings_.manual_env_steps > 0 || settings_.run) {
		ROS_WARN_NAMED("mujoco", "Simulation is currently unpaused. Stepping makes no sense right now.");
		result.success = false;
		action_step_->setPreempted(result);
		action_step_->setSucceeded(result);
		return;
	}

	mujoco_ros_msgs::StepFeedback feedback;

	feedback.steps_left = goal->num_steps + settings_.manual_env_steps;
	settings_.manual_env_steps += goal->num_steps;

	result.success = true;
	while (settings_.manual_env_steps > 0) {
		if (action_step_->isPreemptRequested() || !ros::ok()) {
			ROS_WARN_STREAM_NAMED("mujoco", "Simulation step action preempted");
			result.success = false;
			action_step_->setPreempted(result);
			settings_.manual_env_steps = 0;
			break;
		}

		feedback.steps_left = settings_.manual_env_steps;
		action_step_->publishFeedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	feedback.steps_left = settings_.manual_env_steps;
	action_step_->publishFeedback(feedback);
	action_step_->setSucceeded(result);
}

bool setModelStateCB(mujoco_ros_msgs::SetModelState::Request &req, mujoco_ros_msgs::SetModelState::Response &resp)
{
	uint env_id = (req.state.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg = "Could not find environment with id " + env_id;
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int body_id = mj_name2id(env->model.get(), mjOBJ_BODY, req.state.name.c_str());
	if (body_id == -1) {
		std::string error_msg =
		    "Could not find model (mujoco body) with name " + req.state.name + ". Trying to find geom...";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.state.name.c_str());
		if (geom_id == -1) {
			std::string error_msg = "Could not find model (mujoco geom) with name " + req.state.name;
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
			resp.success        = false;
			return false;
		}
		body_id = env->model->geom_bodyid[geom_id];
	}

	if (env->model->body_jntnum[body_id] > 1) {
		std::string error_msg = "Body " + req.state.name + " has more than one joint ('" +
		                        std::to_string(env->model->body_jntnum[body_id]) +
		                        "'), changes to bodies with more than one joint are not supported!";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int jnt_adr  = env->model->body_jntadr[body_id];
	int jnt_type = env->model->jnt_type[jnt_adr];
	if (jnt_type != mjJNT_FREE) {
		std::string error_msg = "Body " + req.state.name +
		                        " has no joint of type 'freetype'. This service call does not support any other types!";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.success        = false;
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		return false;
	}
	int jnt_qposadr = env->model->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = env->model->jnt_dofadr[jnt_adr];

	geometry_msgs::PoseStamped target_pose;
	geometry_msgs::Twist target_twist;

	// Lock mutex to prevent updating the body while a step is performed
	std::unique_lock<std::mutex> lk(sim_mtx);
	if (req.state.reference_frame != "" && req.state.reference_frame != "world") {
		geometry_msgs::PoseStamped init_pose;
		init_pose.header          = std_msgs::Header();
		init_pose.header.frame_id = req.state.reference_frame;
		init_pose.pose            = req.state.pose;

		try {
			tf_bufferPtr_->transform<geometry_msgs::PoseStamped>(init_pose, target_pose, "world");

			// Only pose can be transformed. Twist will be ignored!
			ROS_WARN_NAMED("mujoco", "Transforming twists from other frames is not supported!");

		} catch (tf2::TransformException &ex) {
			ROS_WARN_STREAM_NAMED("mujoco", ex.what());
			resp.status_message = static_cast<decltype(resp.status_message)>(ex.what());
			resp.success        = false;
			return false;
		}
	} else {
		target_pose.pose = req.state.pose;
		target_twist     = req.state.twist;
	}

	// Set freejoint position and quaternion
	if (req.set_pose) {
		mjtNum quat[4] = { target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y,
			                target_pose.pose.orientation.z };
		mj_normalizeQuat(env->model.get(), quat);

		env->data->qpos[jnt_qposadr]     = target_pose.pose.position.x;
		env->data->qpos[jnt_qposadr + 1] = target_pose.pose.position.y;
		env->data->qpos[jnt_qposadr + 2] = target_pose.pose.position.z;
		env->data->qpos[jnt_qposadr + 3] = quat[0];
		env->data->qpos[jnt_qposadr + 4] = quat[1];
		env->data->qpos[jnt_qposadr + 5] = quat[2];
		env->data->qpos[jnt_qposadr + 6] = quat[3];
	}

	// Set freejoint twist
	if (req.set_twist) {
		env->data->qvel[jnt_dofadr]     = target_twist.linear.x;
		env->data->qvel[jnt_dofadr + 1] = target_twist.linear.y;
		env->data->qvel[jnt_dofadr + 2] = target_twist.linear.z;
		env->data->qvel[jnt_dofadr + 3] = target_twist.angular.x;
		env->data->qvel[jnt_dofadr + 4] = target_twist.angular.y;
		env->data->qvel[jnt_dofadr + 5] = target_twist.angular.z;
	}

	resp.success = true;
	return true;
}

bool getModelStateCB(mujoco_ros_msgs::GetModelState::Request &req, mujoco_ros_msgs::GetModelState::Response &resp)
{
	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg = "Could not find environment with id " + req.env_id;
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int body_id = mj_name2id(env->model.get(), mjOBJ_BODY, req.name.c_str());
	if (body_id == -1) {
		std::string error_msg = "Could not find model (mujoco body) with name " + req.name + ". Trying to find geom...";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.name.c_str());
		if (geom_id == -1) {
			std::string error_msg = "Could not find model (mujoco geom) with name " + req.name;
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
			resp.success        = false;
			return false;
		}
		body_id = env->model->geom_bodyid[geom_id];
	}

	int jnt_adr  = env->model->body_jntadr[body_id];
	int jnt_type = env->model->jnt_type[jnt_adr];
	if (jnt_type != mjJNT_FREE) {
		std::string error_msg =
		    "Body " + req.name + " has no joint of type 'freetype'. This service call does not support any other types!";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.success        = false;
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		return false;
	}
	int jnt_qposadr = env->model->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = env->model->jnt_dofadr[jnt_adr];

	resp.state.pose.position.x    = env->data->qpos[jnt_qposadr];
	resp.state.pose.position.y    = env->data->qpos[jnt_qposadr + 1];
	resp.state.pose.position.z    = env->data->qpos[jnt_qposadr + 2];
	resp.state.pose.orientation.w = env->data->qpos[jnt_qposadr + 3];
	resp.state.pose.orientation.x = env->data->qpos[jnt_qposadr + 4];
	resp.state.pose.orientation.y = env->data->qpos[jnt_qposadr + 5];
	resp.state.pose.orientation.z = env->data->qpos[jnt_qposadr + 6];

	resp.state.twist.linear.x  = env->data->qvel[jnt_dofadr];
	resp.state.twist.linear.y  = env->data->qvel[jnt_dofadr + 1];
	resp.state.twist.linear.z  = env->data->qvel[jnt_dofadr + 2];
	resp.state.twist.angular.x = env->data->qvel[jnt_dofadr + 3];
	resp.state.twist.angular.y = env->data->qvel[jnt_dofadr + 4];
	resp.state.twist.angular.z = env->data->qvel[jnt_dofadr + 5];

	resp.state.reference_frame = "world";

	resp.success = true;
	return true;
}

bool resetBodyQPosCB(mujoco_ros_msgs::ResetBodyQPos::Request &req, mujoco_ros_msgs::ResetBodyQPos::Response &resp)
{
	uint env_id = (req.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg = "Could not find environment with id " + env_id;
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int body_id = mj_name2id(env->model.get(), mjOBJ_BODY, req.name.c_str());
	if (body_id == -1) {
		std::string error_msg = "Could not find model (mujoco body) with name " + req.name + ". Trying to find geom...";
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.name.c_str());
		if (geom_id == -1) {
			std::string error_msg = "Could not find model (mujoco geom) with name " + req.name;
			ROS_WARN_STREAM_NAMED("mujoco", error_msg);
			resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
			resp.success        = false;
			return false;
		}
		body_id = env->model->geom_bodyid[geom_id];
	}

	int jnt_adr = env->model->body_jntadr[body_id];
	int num_jnt = env->model->body_jntnum[body_id];
	mju_copy(env->data->qpos + env->model->jnt_qposadr[jnt_adr], env->model->qpos0 + env->model->jnt_qposadr[jnt_adr],
	         num_jnt);
	return true;
}

bool setGeomPropertiesCB(mujoco_ros_msgs::SetGeomProperties::Request &req,
                         mujoco_ros_msgs::SetGeomProperties::Response &resp)
{
	uint env_id = (req.properties.env_id);
	ROS_DEBUG_STREAM_NAMED("mujoco", "Searching for env '/env" << env_id << "'");
	MujocoEnvPtr env = environments::getEnvById(env_id);

	if (env == nullptr) {
		std::string error_msg = "Could not find environment with id " + env_id;
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int geom_id = mj_name2id(env->model.get(), mjOBJ_GEOM, req.properties.name.c_str());
	if (geom_id == -1) {
		std::string error_msg = "Could not find model (mujoco geom) with name " + req.properties.name;
		ROS_WARN_STREAM_NAMED("mujoco", error_msg);
		resp.status_message = static_cast<decltype(resp.status_message)>(error_msg);
		resp.success        = false;
		return false;
	}

	int body_id = env->model->geom_bodyid[geom_id];

	// Lock mutex to prevent updating the body while a step is performed
	std::unique_lock<std::mutex> lk(sim_mtx);

	ROS_DEBUG_STREAM_NAMED("mujoco", "Changing properties of geom '" << req.properties.name.c_str() << "' ...");
	if (req.set_mass) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "\tReplacing mass '" << env->model->body_mass[body_id] << "' with new mass '"
		                                                      << req.properties.mass << "'");
		env->model->body_mass[body_id] = req.properties.mass;
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
		env->model->geom_size[geom_id * 3 + 1] = req.properties.size_2;

		mj_forward(env->model.get(), env->data.get());
	}

	if (req.set_type || req.set_mass) {
		mjtNum *qpos_tmp = mj_stackAlloc(env->data.get(), env->model->nq);
		mju_copy(qpos_tmp, env->data->qpos, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied current qpos state");
		mj_setConst(env->model.get(), env->data.get());
		ROS_DEBUG_NAMED("mujoco", "Reset constants");
		mju_copy(env->data->qpos, qpos_tmp, env->model->nq);
		ROS_DEBUG_NAMED("mujoco", "Copied qpos state back to data");
	}

	resp.success = true;
	return true;
}

} // end namespace detail

} // end namespace MujocoSim
