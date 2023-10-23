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

#include <ros/ros.h>

#include <mujoco_ros/mujoco_env.h>

#include <mujoco_ros/offscreen_camera.h>

#include <stdexcept>
#include <sstream>

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

namespace {
int MaybeGlfwInit()
{
	static const int is_initialized = []() {
		auto success = Glfw().glfwInit();
		if (success == GLFW_TRUE) {
			std::atexit(Glfw().glfwTerminate);
		}
		return success;
	}();
	return is_initialized;
}
} // namespace

MujocoEnv *MujocoEnv::instance = nullptr;

MujocoEnv::MujocoEnv(const std::string &admin_hash /* = std::string()*/)
{
	if (!ros::param::get("/use_sim_time", settings_.use_sim_time)) {
		ROS_FATAL_NAMED("mujoco", "/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		                          "or false. Also Make sure it is set before starting any node, "
		                          "otherwise nodes might behave unexpectedly.");
		return;
	}

	ROS_DEBUG_COND(!settings_.use_sim_time, "use_sim_time is set to false. Not publishing sim time to /clock!");

	nh_.reset(new ros::NodeHandle("~"));
	ROS_DEBUG_STREAM("New MujocoEnv created");

	ROS_INFO("Using MuJoCo library version %s", mj_versionString());
	if (mjVERSION_HEADER != mj_version()) {
		ROS_WARN_STREAM("Headers and library have different versions (headers: " << mjVERSION_HEADER
		                                                                         << ", library: " << mj_version() << ")");
	}

	if (!admin_hash.empty()) {
		mju::strcpy_arr(settings_.admin_hash, admin_hash.c_str());
	}

	nh_->param<bool>("eval_mode", settings_.eval_mode, false);
	if (settings_.eval_mode) {
		ROS_INFO("Running in evaluation mode. Parsing admin hash...");
		if (!settings_.admin_hash[0]) {
			ROS_ERROR_NAMED("mujoco", "Evaluation mode requires a hash to verify critical operations are allowed. No "
			                          "hash was provided, aborting launch.");
			settings_.exit_request = 1;
			throw std::runtime_error(
			    "Evaluation mode requires a hash to verify critical operations are allowed. No hash was "
			    "provided, aborting launch.");
		}
	} else {
		ROS_INFO("Running in normal training mode.");
	}

	bool no_x;
	nh_->param<bool>("render_offscreen", settings_.render_offscreen, true);
	nh_->param<bool>("no_x", no_x, false);

	if (no_x && settings_.render_offscreen) {
		ROS_WARN("no_x implies offscreen is disabled! Disabling offscreen rendering.");
		settings_.render_offscreen = false;
	}

	bool headless = true;
	nh_->param<bool>("headless", headless, true);
	if (!headless) {
		mjv_makeScene(nullptr, &offscreen_.scn, Viewer::kMaxGeom);
		gui_adapter_ = new mujoco_ros::GlfwAdapter();
	}

	if (settings_.use_sim_time) {
		clock_pub_ = nh_->advertise<rosgraph_msgs::Clock>("/clock", 1);
		publishSimTime(mjtNum(0));
	}

	bool run;
	nh_->param<bool>("unpause", run, true);
	settings_.run = run;
	ROS_INFO_COND(!run, "Starting Simulation in paused mode");

	mjv_defaultScene(&scn_);
	mjv_defaultPerturb(&pert_);

	if (settings_.render_offscreen) {
		MaybeGlfwInit();
		ROS_DEBUG("Starting offscreen render thread");
		offscreen_.render_thread_handle = boost::thread(&MujocoEnv::offscreenRenderLoop, this);
	}

	nh_->param<int>("num_steps", num_steps_until_exit_, -1);
	ROS_INFO_STREAM_COND(num_steps_until_exit_ > 0, "Sim will terminate after " << num_steps_until_exit_ << " steps");

	// init VFS
	mj_defaultVFS(&vfs_);

	setupServices();

	MujocoEnv::instance = this;

	mjcb_control = proxyControlCB;
	mjcb_passive = proxyPassiveCB;

	plugin_utils::initPluginLoader();

	static_broadcaster_ = tf2_ros::StaticTransformBroadcaster();
	tf_bufferPtr_.reset(new tf2_ros::Buffer());
	tf_bufferPtr_->setUsingDedicatedThread(true);
	tf_listenerPtr_.reset(new tf2_ros::TransformListener(*tf_bufferPtr_));
}

void MujocoEnv::registerCollisionFunction(int geom_type1, int geom_type2, mjfCollision collision_cb)
{
	if (custom_collisions_.find(std::pair(geom_type1, geom_type2)) != custom_collisions_.end() &&
	    custom_collisions_.find(std::pair(geom_type2, geom_type2)) != custom_collisions_.end()) {
		ROS_WARN_STREAM_NAMED("mujoco", "A user defined collision callback for collisions between geoms of type "
		                                    << geom_type1 << " and " << geom_type2
		                                    << " have already been registered. This might lead to unexpected behavior!");
	} else {
		custom_collisions_.insert(std::pair(geom_type1, geom_type2));
		defaultCollisionFunctions.emplace_back(
		    CollisionFunctionDefault(geom_type1, geom_type2, mjCOLLISIONFUNC[geom_type1][geom_type2]));
	}
	mjCOLLISIONFUNC[geom_type1][geom_type2] = collision_cb;
}

void MujocoEnv::registerStaticTransform(geometry_msgs::TransformStamped &transform)
{
	ROS_DEBUG_STREAM_NAMED("mujoco", "Registering static transform for frame " << transform.child_frame_id);
	for (auto it = static_transforms_.begin(); it != static_transforms_.end();) {
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

	static_broadcaster_.sendTransform(static_transforms_);
}

void MujocoEnv::eventLoop()
{
	ROS_DEBUG("Starting event loop");
	is_event_running_ = 1;
	auto now          = mujoco_ros::Viewer::Clock::now();
	auto fps_cap      = Seconds(mujoco_ros::Viewer::render_ui_rate_upper_bound_); // Cap at 60 fps
	while (ros::ok() && !settings_.exit_request.load() && (!settings_.headless)) {
		{
			std::unique_lock<std::recursive_mutex> lock(physics_thread_mutex_);
			now = mujoco_ros::Viewer::Clock::now();

			if (settings_.load_request.load() == 1) {
				ROS_DEBUG("Load request received");
				loadWithModelAndData();
				ROS_DEBUG("Done loading");

				mnew = nullptr;
				dnew = nullptr;
				settings_.load_request.store(0);
			} else if (settings_.load_request.load() >= 2) { // Loading mnew and dnew requested
				ROS_DEBUG("Initializing queued model and data");
				if (initModelFromQueue()) {
					ROS_DEBUG("Init for load done. Requesting next load step");
					settings_.load_request.store(1);
				} else {
					ROS_ERROR("Init for load failed. Aborting load request");
					mj_deleteData(dnew);
					mj_deleteModel(mnew);
					mnew = nullptr;
					dnew = nullptr;
					settings_.load_request.store(0);
				}
			}

			if (settings_.reset_request.load()) {
				resetSim();
			}
		}

		std::this_thread::sleep_for(fps_cap - now.time_since_epoch());
	}
	ROS_DEBUG("Closing all connected viewers");
	for (const auto viewer : connected_viewers_) {
		viewer->exit_request.store(1);
	}
	ROS_DEBUG("Exiting event loop");
	is_event_running_ = 0;
}

void MujocoEnv::resetSim()
{
	ROS_DEBUG("Sleeping to ensure all (old) ROS messages are sent");
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	ROS_DEBUG("Resetting simulation environment");

	mj_resetData(this->model_.get(), this->data_.get());
	loadInitialJointStates();
	publishSimTime(this->data_->time);

	for (auto &plugin : plugins_) {
		plugin->safe_reset();
	}

	for (const auto viewer : connected_viewers_) {
		viewer->reset_request.store(1);
	}
	settings_.reset_request.store(0);
}

void MujocoEnv::loadInitialJointStates()
{
	ROS_DEBUG("Fetching and applying initial joint positions ...");

	// Joint positions
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_positions/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_positions/joint_map") && joint_map.empty()) {
		ROS_WARN("Initial joint positions not recognized by rosparam server. Check your config, "
		         "especially values for single axis joints should explicitly provided as string!");
	}
	for (auto const &[name, str_values] : joint_map) {
		ROS_DEBUG_STREAM("Trying to set jointpos of joint " << name << " to values: " << str_values);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			ROS_WARN_STREAM("Joint with name '" << name << "' could not be found. Initial joint position cannot be set!");
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

		auto *axis_vals = new double[num_axes];
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
			ROS_ERROR_STREAM("Provided initial position values for joint "
			                 << name << " don't match the degrees of freedom of the joint (exactly " << num_axes
			                 << " values are needed)!");
			delete[] axis_vals;
			continue;
		}
		for (jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			setJointPosition(axis_vals[jnt_axis], id, jnt_axis);
		}
		delete[] axis_vals;

		// Apply changes in forward dynamics
		mj_forward(model_.get(), data_.get());
	}

	// Joint velocities
	joint_map.clear();
	nh_->getParam("initial_joint_velocities/joint_map", joint_map);
	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_velocities/joint_map") && joint_map.empty()) {
		ROS_WARN_NAMED("mujoco", "Initial joint positions not recognized by rosparam server. Check your config, "
		                         "especially values for single axis joints should explicitly provided as string!");
	}
	for (auto const &[name, str_values] : joint_map) {
		ROS_DEBUG_STREAM_NAMED("mujoco", "Trying to set jointvels of joint " << name << " to values: " << str_values);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			ROS_WARN_STREAM_NAMED("mujoco", "Joint with name '"
			                                    << name << "' could not be found. Initial joint velocity cannot be set!");
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

		auto *axis_vals = new double[num_axes];
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
			delete[] axis_vals;
			continue;
		}
		for (jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			setJointVelocity(axis_vals[jnt_axis], id, jnt_axis);
		}
		delete[] axis_vals;
	}
}

void MujocoEnv::setJointPosition(const double &pos, const int &joint_id, const int &jnt_axis /*= 0*/)
{
	data_->qpos[model_->jnt_qposadr[joint_id] + jnt_axis]        = pos;
	data_->qvel[model_->jnt_dofadr[joint_id] + jnt_axis]         = 0;
	data_->qfrc_applied[model_->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void MujocoEnv::setJointVelocity(const double &vel, const int &joint_id, const int &jnt_axis /*= 0*/)
{
	data_->qvel[model_->jnt_dofadr[joint_id] + jnt_axis]         = vel;
	data_->qfrc_applied[model_->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void MujocoEnv::completeEnvSetup()
{
	loadInitialJointStates();

	ROS_DEBUG("Resetting noise ...");
	free(ctrlnoise_);
	ctrlnoise_ = static_cast<mjtNum *>(mju_malloc(sizeof(mjtNum) * static_cast<size_t>(model_->nu)));
	mju_zero(ctrlnoise_, model_->nu);

	loadPlugins();
	ROS_DEBUG("Env setup complete");
}

void MujocoEnv::loadPlugins()
{
	ROS_DEBUG("Loading MujocoRosPlugins ...");
	cb_ready_plugins_.clear();
	cb_ready_plugins_.shrink_to_fit();

	XmlRpc::XmlRpcValue plugin_config;
	if (plugin_utils::parsePlugins(nh_.get(), plugin_config)) {
		plugin_utils::registerPlugins(nh_->getNamespace(), plugin_config, plugins_, this);
	}

	for (const auto &plugin : plugins_) {
		if (plugin->safe_load(model_.get(), data_.get())) {
			cb_ready_plugins_.push_back(plugin);
		}
	}
	ROS_DEBUG("Done loading MujocoRosPlugins");
}

void MujocoEnv::physicsLoop()
{
	ROS_DEBUG("Physics loop started");
	is_physics_running_ = 1;
	// CPU-sim syncronization point
	std::chrono::time_point<mujoco_ros::Viewer::Clock> syncCPU;
	mjtNum syncSim = 0;

	// run until asked to exit
	while (ros::ok() && !settings_.exit_request.load() && num_steps_until_exit_ != 0) {
		// Sleep for 1 ms or yield, to let the main thread run
		// yield results in busy wait - which has better timing but kills battery life
		if (settings_.run.load() && settings_.busywait) {
			std::this_thread::yield();
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		// Run only if model is present
		if (model_) {
			if (physics_thread_mutex_.try_lock()) {
				// lock the sim mutex
				// std::lock_guard<std::recursive_mutex> lock(physics_thread_mutex_);
				// Running
				if (settings_.run.load()) {
					// record CPU time at start of iteration
					const auto startCPU = mujoco_ros::Viewer::Clock::now();

					// Elapsed CPU and simulation time since last sync
					const auto elapsedCPU = startCPU - syncCPU;
					double elapsedSim     = data_->time - syncSim;

					// inject noise
					if (ctrl_noise_std) {
						// Convert rate and scale to discrete time (Ornstein-Uhlenbeck)
						mjtNum rate  = mju_exp(-model_->opt.timestep / mju_max(ctrl_noise_rate, mjMINVAL));
						mjtNum scale = ctrl_noise_std * mju_sqrt(1 - rate * rate);

						for (int i = 0; i < model_->nu; i++) {
							// Update noise
							ctrlnoise_[i] = rate * ctrlnoise_[i] + scale * mju_standardNormal(nullptr);

							// Apply noise
							data_->ctrl[i] = ctrlnoise_[i];
						}
					}

					// Requested slow-down factor
					double slowdown = 100 / percentRealTime[settings_.real_time_index];

					// Misalignment condition: distance from target sim time is bigger than syncsimalign
					bool misaligned = mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

					// Out-of-sync (for any reason): reset sync times, step
					if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
					    settings_.speed_changed) {
						// re-sync
						syncCPU                 = startCPU;
						syncSim                 = data_->time;
						settings_.speed_changed = false;

						// run single step, let next iteration deal with timing
						mj_step(model_.get(), data_.get());
						publishSimTime(data_->time);
						runLastStageCbs();
						if (settings_.render_offscreen) {
							// Wait until no render request is pending
							while (offscreen_.request_pending.load()) {
								std::this_thread::sleep_for(std::chrono::milliseconds(3));
							}
							std::unique_lock<std::mutex> lock(offscreen_.render_mutex);

							for (const auto &cam_ptr : offscreen_.cams) {
								if (cam_ptr->shouldRender(ros::Time(data_->time))) {
									mjv_updateSceneState(model_.get(), data_.get(), &cam_ptr->vopt_, &cam_ptr->scn_state_);
									runRenderCbs(&cam_ptr->scn_state_.plugincache);
									offscreen_.request_pending.store(true);
								}
							}
						}
						offscreen_.cond_render_request.notify_one();

						if (num_steps_until_exit_ > 0) {
							num_steps_until_exit_--;
						}
					}

					// In-sync: step until ahead of CPU
					else {
						bool measured  = false;
						mjtNum prevSim = data_->time;

						// If real-time is bound, run until sim steps are in sync with CPU steps, otherwise run as fast as
						// possible
						while ((settings_.real_time_index == 0 ||
						        Seconds((data_->time - syncSim) * slowdown) < mujoco_ros::Viewer::Clock::now() - syncCPU) &&
						       (mujoco_ros::Viewer::Clock::now() - startCPU <
						            Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_) &&
						        !connected_viewers_.empty()) && // only break if rendering UI is actually necessary
						       !settings_.exit_request.load() &&
						       num_steps_until_exit_ != 0) {
							// measure slowdown before first step
							if (!measured && elapsedSim) {
								if (settings_.real_time_index != 0) {
									sim_state_.measured_slowdown =
									    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
									measured = true;
								} else if (syncCPU.time_since_epoch().count() % 3 == 0) { // measure slowdown every 3rd step for
									                                                       // an updated estimate
									sim_state_.measured_slowdown =
									    std::chrono::duration<double>(mujoco_ros::Viewer::Clock::now() - syncCPU).count() /
									    Seconds(data_->time - syncSim).count();
								}
							}

							// Call mj_step
							mj_step(model_.get(), data_.get());
							publishSimTime(data_->time);
							runLastStageCbs();
							if (settings_.render_offscreen) {
								// Wait until no render request is pending
								while (offscreen_.request_pending.load()) {
									std::this_thread::sleep_for(std::chrono::milliseconds(5));
								}
								std::unique_lock<std::mutex> lock(offscreen_.render_mutex);

								for (const auto &cam_ptr : offscreen_.cams) {
									if (cam_ptr->shouldRender(ros::Time(data_->time))) {
										mjv_updateSceneState(model_.get(), data_.get(), &cam_ptr->vopt_, &cam_ptr->scn_state_);
										runRenderCbs(&cam_ptr->scn_state_.plugincache);
										offscreen_.request_pending.store(true);
									}
								}
							}
							offscreen_.cond_render_request.notify_one();

							if (num_steps_until_exit_ > 0) {
								num_steps_until_exit_--;
							}

							// Break if reset
							if (data_->time < prevSim) {
								break;
							}
						}
					}
				}
				// Paused
				else {
					if (settings_.env_steps_request.load() > 0) { // Action call or arrow keys used for stepping
						syncSim             = data_->time;
						const auto startCPU = mujoco_ros::Viewer::Clock::now();

						while (settings_.env_steps_request.load() > 0 &&
						       (connected_viewers_.empty() || mujoco_ros::Viewer::Clock::now() - startCPU <
						                                          Seconds(mujoco_ros::Viewer::render_ui_rate_lower_bound_))) {
							// Run single step
							mj_step(model_.get(), data_.get());
							publishSimTime(data_->time);
							runLastStageCbs();
							if (settings_.render_offscreen) {
								// Wait until no render request is pending
								while (offscreen_.request_pending.load()) {
									std::this_thread::sleep_for(std::chrono::milliseconds(3));
								}
								std::unique_lock<std::mutex> lock(offscreen_.render_mutex);

								for (const auto &cam_ptr : offscreen_.cams) {
									if (cam_ptr->shouldRender(ros::Time(data_->time))) {
										mjv_updateSceneState(model_.get(), data_.get(), &cam_ptr->vopt_, &cam_ptr->scn_state_);
										runRenderCbs(&cam_ptr->scn_state_.plugincache);
										offscreen_.request_pending.store(true);
									}
								}
							}
							offscreen_.cond_render_request.notify_one();

							settings_.env_steps_request.fetch_sub(1); // Decrement requested steps counter
							// Break if reset
							if (data_->time < syncSim) {
								break;
							}
						}
					} else {
						// Run mj_forward, to update rendering and joint sliders
						mj_forward(model_.get(), data_.get());
						publishSimTime(data_->time);
					}
				}
				physics_thread_mutex_.unlock();
			} // Release physics_mutex
		}
	}
	is_physics_running_ = 0;
	ROS_INFO_COND(num_steps_until_exit_ == 0, "Reached requested number of steps. Exiting simulation");
	// settings_.exit_request.store(1);
	if (offscreen_.render_thread_handle.joinable()) {
		offscreen_.request_pending.store(true);
		offscreen_.cond_render_request.notify_one();
		ROS_DEBUG("Joining offscreen render thread");
		offscreen_.render_thread_handle.join();
	}
	ROS_DEBUG("Exiting physics loop");
}

void MujocoEnv::startPhysicsLoop()
{
	physics_thread_handle_ = boost::thread(&MujocoEnv::physicsLoop, this);
}

void MujocoEnv::startEventLoop()
{
	event_thread_handle_ = boost::thread(&MujocoEnv::eventLoop, this);
}

void MujocoEnv::waitForPhysicsJoin()
{
	if (physics_thread_handle_.joinable()) {
		physics_thread_handle_.join();
	}
}

void MujocoEnv::waitForEventsJoin()
{
	if (event_thread_handle_.joinable()) {
		event_thread_handle_.join();
	}
}

void MujocoEnv::connectViewer(Viewer *viewer)
{
	if (!connected_viewers_.empty()) {
		ROS_INFO("Connected first viewer, disabling headless mode");
		settings_.headless = false;
	}
	if (std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer) == connected_viewers_.end()) {
		connected_viewers_.push_back(viewer);
		if (sim_state_.model_valid) {
			viewer->mnew_ = model_;
			viewer->dnew_ = data_;
			mju::strcmp_arr(viewer->filename, filename_);
			viewer->loadrequest = true;
		}
		return;
	}
	ROS_WARN("Viewer already connected!");
}

void MujocoEnv::disconnectViewer(Viewer *viewer)
{
	auto it = std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer);
	if (it != connected_viewers_.end()) {
		connected_viewers_.erase(it);
	} else {
		ROS_WARN("Viewer not connected!");
	}

	if (connected_viewers_.empty()) {
		ROS_INFO_COND(settings_.exit_request == 0, "Disconnected last viewer, enabling headless mode");
		settings_.headless = true;
	}
}

void MujocoEnv::publishSimTime(mjtNum time)
{
	if (!settings_.use_sim_time) {
		return;
	}
	// This is the fastes option for intra-node time updates
	// however, together with non-blocking publish it breaks stuff
	// ros::Time::setNow(ros::Time(time));
	rosgraph_msgs::ClockPtr ros_time(new rosgraph_msgs::Clock);
	ros_time->clock.fromSec(time);
	clock_pub_.publish(ros_time);
	// Current workaround to simulate a blocking time update
	while (ros::Time::now() < ros::Time(time)) {
		std::this_thread::yield();
	}
}

bool MujocoEnv::togglePaused(bool paused, const std::string &admin_hash /*= std::string*/)
{
	ROS_DEBUG("Trying to toggle pause");
	if (settings_.eval_mode && paused) {
		ROS_DEBUG("Evaluation mode is active. Checking request validity");
		if (settings_.admin_hash != admin_hash) {
			ROS_ERROR("Unauthorized pause request detected. Ignoring request");
			return false;
		}
		ROS_DEBUG("Request valid. Handling request");
	}
	settings_.run.store(!paused);
	if (settings_.run.load())
		settings_.env_steps_request.store(0);
	return true;
}

const std::vector<MujocoPluginPtr> MujocoEnv::getPlugins()
{
	return this->plugins_;
}

void MujocoEnv::notifyGeomChanged(const int geom_id)
{
	for (const auto &plugin : this->cb_ready_plugins_) {
		plugin->onGeomChanged(this->model_.get(), this->data_.get(), geom_id);
	}
}

int MujocoEnv::getOperationalStatus()
{
	return mju_max(settings_.load_request.load(), mju_max(settings_.visual_init_request, settings_.reset_request));
}

void MujocoEnv::loadWithModelAndData()
{
	model_.reset(mnew, mj_deleteModel);
	data_.reset(dnew, mj_deleteData);

	prepareReload();
	completeEnvSetup();

	ROS_DEBUG("Delegating model loading to viewers");
	for (const auto viewer : connected_viewers_) {
		viewer->Load(model_, data_, filename_);
	}

	if (settings_.render_offscreen) {
		ROS_DEBUG("Issuing (re)initialization of offscreen rendering resources");
		settings_.visual_init_request.store(1);
		while (settings_.visual_init_request.load() != 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			offscreen_.cond_render_request.notify_one();
		}
	}

	load_error_[0]         = '\0';
	sim_state_.model_valid = true;
}

bool MujocoEnv::initModelFromQueue()
{
	// clear previous error message
	load_error_[0] = '\0';

	bool is_new = false;
	if (queued_filename_[0]) {
		is_new = mju::strcmp_arr(filename_, queued_filename_);
	}

	if (!is_new) {
		mju::strcpy_arr(queued_filename_, filename_);
	}

	bool is_mjb = false;
	if (mju::strlen_arr(queued_filename_) > 4 &&
	    !std::strncmp(queued_filename_ + mju::strlen_arr(queued_filename_) - 4, ".mjb",
	                  mju::sizeof_arr(queued_filename_) - mju::strlen_arr(queued_filename_) + 4)) {
		is_mjb = true;
	}

	bool is_file = false;
	if (mju::strlen_arr(queued_filename_) > 4 &&
	    !std::strncmp(queued_filename_ + mju::strlen_arr(queued_filename_) - 4, ".xml",
	                  mju::sizeof_arr(queued_filename_) - mju::strlen_arr(queued_filename_) + 4)) {
		is_file = true;
	} else {
		try {
			is_file = boost::filesystem::is_regular_file(queued_filename_);
		} catch (const boost::filesystem::filesystem_error &ex) {
			ROS_DEBUG_STREAM("\tFilesystem error while checking for regular file: " << ex.what());
		}
	}

	int vfs_id              = mj_findFileVFS(&vfs_, "model_string");
	size_t filecontent_size = 0;
	if (vfs_id > -1) {
		filecontent_size = static_cast<size_t>(vfs_.filesize[vfs_id]);
	}

	char *filedata_backup = new char[filecontent_size];

	if (is_file) {
		ROS_DEBUG("\tModel is a regular file. Loading from filesystem");
	} else if (queued_filename_[0] != '\0') { // new model string
		ROS_DEBUG("\tModel is not a regular file. Loading from string");

		ROS_WARN("Loading nested resources (textures, meshes, ...) from string is broken since 2.3.4. A fix is on the "
		         "way (see https://github.com/deepmind/mujoco/discussions/957#discussion-5348269)");

		if (vfs_id > -1) {
			ROS_DEBUG("\tBacking up old string content");
			memcpy(filedata_backup, vfs_.filedata[vfs_id], filecontent_size);
		}

		mju::strcpy_arr(queued_filename_, "model_string");
		mj_deleteFileVFS(&vfs_, "model_string");
		mj_makeEmptyFileVFS(&vfs_, "model_string", mju::strlen_arr(queued_filename_));
		int file_idx = mj_findFileVFS(&vfs_, "model_string");
		memcpy(vfs_.filedata[file_idx], queued_filename_, mju::strlen_arr(queued_filename_));
		ROS_DEBUG("\tSaved string content to VFS");
	}

	if (is_mjb) {
		ROS_DEBUG("\tLoading mjb file");
		mnew = mj_loadModel(queued_filename_, nullptr);
	} else {
		if (is_file) {
			ROS_DEBUG("\tLoading xml file");
			mnew = mj_loadXML(queued_filename_, nullptr, load_error_, kErrorLength);
		} else {
			ROS_DEBUG("\tLoading virtual file from VFS");
			mnew = mj_loadXML(queued_filename_, &vfs_, load_error_, kErrorLength);
		}
	}

	for (const auto viewer : connected_viewers_) {
		mju::strcpy_arr(viewer->load_error, load_error_);
	}

	if (!mnew) {
		ROS_ERROR_STREAM("Loading model from file failed: " << load_error_);
		ROS_DEBUG("\tRolling back old model");

		if (!is_file && filecontent_size > 0) {
			mj_deleteFileVFS(&vfs_, "model_string");
			mj_makeEmptyFileVFS(&vfs_, "model_string", filecontent_size);
			int file_idx = mj_findFileVFS(&vfs_, "model_string");
			memcpy(vfs_.filedata[file_idx], filedata_backup, filecontent_size);
			ROS_DEBUG("\tRestored string content to VFS");
		}
		delete[] filedata_backup;

		// 'clear' new filename
		queued_filename_[0] = '\0';

		sim_state_.model_valid = false;
		return false;
	}

	ROS_DEBUG("Model compiled successfully");
	dnew = mj_makeData(mnew);

	// 'clear' filename in queue
	mju::strcpy_arr(filename_, queued_filename_);
	queued_filename_[0] = '\0';
	// delete allocated memory for VFS backup
	delete[] filedata_backup;

	// Compiler warning: print and pause
	if (load_error_[0]) {
		// next mj_forward will print the message
		ROS_WARN_STREAM("Model compiled, but got simulation warning: " << load_error_);
		if (!settings_.headless)
			settings_.run = 0;
	}

	// Update real-time settings
	int num_clicks  = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
	float min_error = 1e6f;
	float desired;
	nh_->param<float>("realtime", desired, mnew->vis.global.realtime);

	if (desired == -1.f) {
		settings_.real_time_index = 0;
	} else if (desired <= 0.f or desired > 1.f) {
		ROS_WARN("Desired realtime should be in range (0, 1]. Falling back to default (1)");
		settings_.real_time_index = 1;
	} else {
		desired = mju_log(100 * desired);
		for (int click = 0; click < num_clicks; click++) {
			float error = mju_abs(mju_log(percentRealTime[click]) - desired);
			if (error < min_error) {
				min_error                 = error;
				settings_.real_time_index = click;
			}
		}
	}

	return true;
}

bool MujocoEnv::step(int num_steps /* = 1*/, bool blocking /* = true*/)
{
	if (!model_) {
		ROS_ERROR("No model loaded. Cannot step");
		return false;
	}

	if (settings_.run) {
		ROS_WARN("Simulation is already running. Ignoring request");
		return false;
	}

	if (num_steps <= 0) {
		ROS_WARN("Number of steps must be positive. Ignoring request");
		return false;
	}

	if (blocking && boost::this_thread::get_id() == physics_thread_handle_.get_id()) {
		ROS_WARN("Simulation is running in the same thread. Cannot block! Ignoring request");
		return false;
	}

	ROS_DEBUG("Handling request of stepping %d steps", num_steps);
	settings_.env_steps_request.store(num_steps);
	if (blocking) {
		ROS_DEBUG("\t blocking until steps are done");
		while (settings_.env_steps_request.load() > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	return true;
}

void MujocoEnv::prepareReload()
{
	ROS_DEBUG("\tResetting collision cbs to default");
	for (const auto func : defaultCollisionFunctions) {
		mjCOLLISIONFUNC[func.geom_type1_][func.geom_type2_] = func.collision_cb_;
	}
	defaultCollisionFunctions.clear();
	custom_collisions_.clear();

	offscreen_.rgb.reset();
	offscreen_.depth.reset();
	cb_ready_plugins_.clear();
	plugins_.clear();
	offscreen_.cams.clear();
}

MujocoEnv::~MujocoEnv()
{
	ROS_DEBUG("Destructor called");
	connected_viewers_.clear();
	free(this->ctrlnoise_);
	this->cb_ready_plugins_.clear();
	this->plugins_.clear();
	mj_deleteVFS(&vfs_);

	plugin_utils::unloadPluginloader();
}

} // namespace mujoco_ros
