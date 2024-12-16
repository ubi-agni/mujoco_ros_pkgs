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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/array_safety.h>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#else // MJR_ROS_VERSION == ROS_2
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#endif

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

bool MujocoEnv::TogglePaused(bool paused, const std::string &admin_hash /*= std::string*/)
{
	MJR_DEBUG("Trying to toggle pause");
	if (!VerifyAdminHash(admin_hash)) {
		MJR_ERROR("Unauthorized pause request detected. Ignoring request");
		return false;
	}
	settings_.settings_changed.store(1);
	settings_.run.store(!paused);
	if (settings_.run.load())
		settings_.env_steps_request.store(0);
	return true;
}

int MujocoEnv::GetOperationalStatus()
{
	return mju_max(settings_.load_request.load(), mju_max(settings_.visual_init_request, settings_.reset_request));
}

bool MujocoEnv::Step(int num_steps /* = 1*/, bool blocking /* = true*/)
{
	if (!model_) {
		MJR_ERROR("No model loaded. Cannot step");
		return false;
	}

	if (settings_.run) {
		MJR_WARN("Simulation is already running. Ignoring request");
		return false;
	}

	if (num_steps <= 0) {
		MJR_WARN("Number of steps must be positive. Ignoring request");
		return false;
	}

	if (blocking && std::this_thread::get_id() == physics_thread_handle_.get_id()) {
		MJR_WARN("Simulation is running in the same thread. Cannot block! Ignoring request");
		return false;
	}

	MJR_DEBUG("Handling request of stepping %d steps", num_steps);
	settings_.env_steps_request.store(num_steps);
	if (blocking) {
		MJR_DEBUG("\t blocking until steps are done");
		while (settings_.env_steps_request.load() > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	return true;
}

void MujocoEnv::UpdateModelFlags(const mjOption *opt)
{
	std::unique_lock<std::recursive_mutex> lock(physics_thread_mutex_);
	// Physics flags
	model_->opt.disableflags = opt->disableflags;
	model_->opt.enableflags  = opt->enableflags;

	// Enabled actuator groups
	model_->opt.disableactuator = opt->disableactuator;
}

void MujocoEnv::SetJointPosition(const double &pos, const int &joint_id, const int &jnt_axis /*= 0*/)
{
	data_->qpos[model_->jnt_qposadr[joint_id] + jnt_axis]        = pos;
	data_->qvel[model_->jnt_dofadr[joint_id] + jnt_axis]         = 0;
	data_->qfrc_applied[model_->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void MujocoEnv::SetJointVelocity(const double &vel, const int &joint_id, const int &jnt_axis /*= 0*/)
{
	data_->qvel[model_->jnt_dofadr[joint_id] + jnt_axis]         = vel;
	data_->qfrc_applied[model_->jnt_dofadr[joint_id] + jnt_axis] = 0;
}

void MujocoEnv::Reset()
{
	MJR_DEBUG("Reset requested");
	settings_.reset_request.store(1);

	while (GetOperationalStatus() > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

void MujocoEnv::Shutdown()
{
	MJR_DEBUG("Shutdown requested");
	settings_.exit_request.store(1);
}

bool MujocoEnv::LoadModelFromString(const std::string &model, char *load_error, const int error_sz)
{
	MJR_DEBUG("Loading model from string requested");
	if (model.size() > kMaxFilenameLength) {
		MJR_ERROR_STREAM("Model string is too long. Max length: " << kMaxFilenameLength << " (got " << model.size()
		                                                          << ")");
		return false;
	}
	mju::strcpy_arr(queued_filename_, model.c_str());

	MJR_DEBUG("Issuing model load");
	settings_.load_request.store(2);
	while (GetOperationalStatus() > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	if (load_error != nullptr) {
		std::strncpy(load_error, load_error_, error_sz);
	}
	return sim_state_.model_valid;
}

bool MujocoEnv::SetBodyState(const std::string &body_name, mjtNum *pose, mjtNum *twist, mjtNum &mass, bool set_pose,
                             bool set_twist, bool set_mass, bool reset_qpos, const std::string &admin_hash,
                             char *status_message, const int status_sz)
{
	MJR_DEBUG("Setting body state requested");

	std::string message = "";
	if (!VerifyAdminHash(admin_hash)) {
		message = "Unauthorized body state request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	if (body_name.empty()) {
		message = "Body name is empty. Cannot set body state";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int body_id = mj_name2id(model_.get(), mjOBJ_BODY, body_name.c_str());
	if (body_id == -1) {
		MJR_WARN_STREAM("Could not find body with name " << body_name << ". Trying to find geom...");
		int geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, body_name.c_str());
		if (geom_id == -1) {
			message = "Could not find model (not body nor geom) with name " + body_name;
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		}
		body_id = model_->geom_bodyid[geom_id];
		MJR_WARN_STREAM("Found body named '" << mj_id2name(model_.get(), mjOBJ_BODY, body_id) << "' as parent of geom '"
		                                     << body_name << "'");
	}

	if (set_mass) {
		RecursiveLock sim_lock(physics_thread_mutex_);
		MJR_DEBUG_STREAM("\tReplacing mass '" << model_->body_mass[body_id] << "' with new mass '" << mass << "'");
		model_->body_mass[body_id] = mass;

		// Prevent rendering the reset to q0
		MutexLock render_lock(offscreen_.render_mutex);
		mj_markStack(data_.get());
		mjtNum *qpos_tmp = mj_stackAllocNum(data_.get(), model_->nq);
		mju_copy(qpos_tmp, data_->qpos, model_->nq);
		MJR_DEBUG("Copied current qpos state");
		mj_setConst(model_.get(), data_.get());
		MJR_DEBUG("Reset constants because of mass change");
		mju_copy(data_->qpos, qpos_tmp, model_->nq);
		MJR_DEBUG("Copied qpos state back to data");
		mj_freeStack(data_.get());
	}

	int jnt_adr     = model_->body_jntadr[body_id];
	int jnt_type    = model_->jnt_type[jnt_adr];
	int num_jnt     = model_->body_jntnum[body_id];
	int jnt_qposadr = model_->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = model_->jnt_dofadr[jnt_adr];

	if (set_pose || set_twist || reset_qpos) {
		if (jnt_adr == -1) {
			message = "Body has no joints. Cannot move body";
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		} else if (jnt_type != mjJNT_FREE) {
			message = "Body " + body_name +
			          " has no joint of type 'freetype'. This service call does not support any other types!";
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		} else if (num_jnt > 1) {
			message = "Body " + body_name + " has more than one joint ('" + std::to_string(model_->body_jntnum[body_id]) +
			          "'). Pose/twist changes to bodies with more than one joint are not supported!";
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		}

		// Lock mutex to prevent updating the body while a step is performed
		RecursiveLock sim_lock(physics_thread_mutex_);
		if (reset_qpos) {
			MJR_WARN_COND(set_pose,
			              "set_pose and reset_qpos were both passed. reset_qpos will overwrite the custom pose!");
			set_pose = false;
			mju_copy(data_->qpos + jnt_qposadr, model_->qpos0 + jnt_qposadr,
			         7); // 7 is the number of degrees of freedom of a freejoint
			if (!set_twist) {
				// Reset twist to zero if not set
				mju_zero(data_->qvel + jnt_dofadr, 6);
			}
		}
		if (set_pose) {
			MJR_DEBUG_STREAM("\tSetting pose of body '" << body_name << "'"
			                                            << " to " << pose[0] << ", " << pose[1] << ", " << pose[2] << ", "
			                                            << pose[3] << ", " << pose[4] << ", " << pose[5] << ", "
			                                            << pose[6]);
			mju_copy(data_->qpos + jnt_qposadr, pose, 7);
		}

		if (set_twist) {
			MJR_DEBUG_STREAM("\tSetting twist of body '" << body_name << "'"
			                                             << " to " << twist[0] << ", " << twist[1] << ", " << twist[2]
			                                             << ", " << twist[3] << ", " << twist[4] << ", " << twist[5]);
			mju_copy(data_->qvel + jnt_dofadr, twist, 6);
		}
	}
	return true;
}

bool MujocoEnv::GetBodyState(std::string &body_name, mjtNum *pose, mjtNum *twist, mjtNum *mass,
                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	std::string message = "";
	MJR_DEBUG("Getting body state requested");
	if (!VerifyAdminHash(admin_hash)) {
		message = "Unauthorized body state request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	if (body_name.empty()) {
		message = "Body name is empty. Cannot get body state";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int body_id = mj_name2id(model_.get(), mjOBJ_BODY, body_name.c_str());
	if (body_id == -1) {
		MJR_WARN_STREAM("Could not find body with name " << body_name << ". Trying to find geom...");
		int geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, body_name.c_str());
		if (geom_id == -1) {
			message = "Could not find model (not body nor geom) with name " + body_name;
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		}
		body_id = model_->geom_bodyid[geom_id];
		MJR_WARN_STREAM("Found body named '" << mj_id2name(model_.get(), mjOBJ_BODY, body_id) << "' as parent of geom '"
		                                     << body_name << "'");
		body_name = mj_id2name(model_.get(), mjOBJ_BODY, body_id);
	}

	*mass = model_->body_mass[body_id];

	int jnt_adr     = model_->body_jntadr[body_id];
	int jnt_type    = model_->jnt_type[jnt_adr];
	int num_jnt     = model_->body_jntnum[body_id];
	int jnt_qposadr = model_->jnt_qposadr[jnt_adr];
	int jnt_dofadr  = model_->jnt_dofadr[jnt_adr];

	// Stop sim to get coherent data
	RecursiveLock sim_lock(physics_thread_mutex_);
	if (jnt_adr == -1 || jnt_type != mjJNT_FREE || num_jnt > 1) {
		mju_copy(pose, data_->xpos + body_id * 3, 3);
		mju_copy(pose + 3, data_->xquat + body_id * 4, 4);
		mju_copy(twist, data_->cvel + body_id * 6, 6);
	} else {
		mju_copy(pose, data_->qpos + jnt_qposadr, 7);
		mju_copy(twist, data_->qvel + jnt_dofadr, 6);
	}

	return true;
}

bool MujocoEnv::SetGravity(const mjtNum *gravity, const std::string &admin_hash, char *status_message,
                           const int status_sz)
{
	MJR_DEBUG("Setting gravity requested");
	if (!VerifyAdminHash(admin_hash)) {
		std::string message = "Unauthorized gravity request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	// Lock mutex to prevent updating gravity during a step
	RecursiveLock sim_lock(physics_thread_mutex_);
	mju_copy(model_->opt.gravity, gravity, 3);
	return true;
}

bool MujocoEnv::GetGravity(mjtNum *gravity, const std::string &admin_hash, char *status_message, const int status_sz)
{
	MJR_DEBUG("Getting gravity requested");
	if (!VerifyAdminHash(admin_hash)) {
		std::string message = "Unauthorized gravity request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	// Lock mutex to get data within one step
	RecursiveLock sim_lock(physics_thread_mutex_);
	mju_copy(gravity, model_->opt.gravity, 3);
	return true;
}

bool MujocoEnv::SetGeomProperties(const std::string &geom_name, const mjtNum body_mass, const mjtNum friction_slide,
                                  const mjtNum friction_spin, const mjtNum friction_roll, const mjtNum size_x,
                                  const mjtNum size_y, const mjtNum size_z, const mjtNum type, bool set_mass,
                                  bool set_friction, bool set_type, bool set_size, const std::string &admin_hash,
                                  char *status_message, const int status_sz)
{
	std::string message = "";
	MJR_DEBUG("Setting geom properties requested");
	if (!VerifyAdminHash(admin_hash)) {
		message = "Unauthorized geom properties request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	if (geom_name.empty()) {
		message = "Geom name is empty. Cannot set geom properties";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, geom_name.c_str());
	if (geom_id == -1) {
		message = "Could not find model (mujoco geom) with name " + geom_name;
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int body_id = model_->geom_bodyid[geom_id];

	MJR_DEBUG_STREAM("\tSetting properties of geom '" << geom_name << "'");
	// Lock mutex to prevent updating the geom while a step is performed
	RecursiveLock sim_lock(physics_thread_mutex_);

	if (set_mass) {
		MJR_DEBUG_STREAM("\tReplacing mass '" << model_->body_mass[body_id] << "' with new mass '" << body_mass << "'");
		model_->body_mass[body_id] = body_mass;
	}
	if (set_friction) {
		MJR_DEBUG_STREAM("\tSetting friction of geom '" << geom_name << "' to " << friction_slide << ", " << friction_spin
		                                                << ", " << friction_roll);
		model_->geom_friction[geom_id * 3]     = friction_slide;
		model_->geom_friction[geom_id * 3 + 1] = friction_spin;
		model_->geom_friction[geom_id * 3 + 2] = friction_roll;
	}
	if (set_type) {
		// Check if type is valid (any of mjtGeom)
		if (type < 0 || type >= mjNGEOMTYPES) {
			message = "Invalid geom type '" + std::to_string(type) + "'. Must be in range [0, " +
			          std::to_string(mjNGEOMTYPES) + ")";
			MJR_ERROR_STREAM(message);
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
			return false;
		}

		MJR_DEBUG_STREAM("\tSetting type of geom '" << geom_name << "' to " << type);
		model_->geom_type[geom_id] = type;
	}
	if (set_size) {
		if (size_x > model_->geom_size[geom_id * 3] || size_y > model_->geom_size[geom_id * 3 + 1] ||
		    size_z > model_->geom_size[geom_id * 3 + 2]) {
			message = "New geom size is larger than the current size. AABBs are not recomputed, this may cause incorrect "
			          "collisions!";
			MJR_WARN(message.c_str());
			if (status_message != nullptr) {
				std::strncpy(status_message, message.c_str(), status_sz);
			}
		}
		MJR_DEBUG_STREAM("\tSetting size of geom '" << geom_name << "' to " << size_x << ", " << size_y << ", "
		                                            << size_z);
		model_->geom_size[geom_id * 3]     = size_x;
		model_->geom_size[geom_id * 3 + 1] = size_y;
		model_->geom_size[geom_id * 3 + 2] = size_z;

		mj_forward(model_.get(), data_.get());
	}

	if (set_type || set_mass) {
		// Prevent rendering the reset to q0
		MutexLock render_lock(offscreen_.render_mutex);
		mj_markStack(data_.get());
		mjtNum *qpos_tmp = mj_stackAllocNum(data_.get(), model_->nq);
		mju_copy(qpos_tmp, data_->qpos, model_->nq);
		MJR_DEBUG("Copied current qpos state");
		mj_setConst(model_.get(), data_.get());
		MJR_DEBUG("Reset constants because of geom property change");
		mju_copy(data_->qpos, qpos_tmp, model_->nq);
		MJR_DEBUG("Copied qpos state back to data");
		mj_freeStack(data_.get());
	}

	NotifyGeomChanged(geom_id);
	return true;
}

bool MujocoEnv::GetGeomProperties(const std::string &geom_name, mjtNum &body_mass, mjtNum &friction_slide,
                                  mjtNum &friction_spin, mjtNum &friction_roll, mjtNum &size_x, mjtNum &size_y,
                                  mjtNum &size_z, mjtNum &type, const std::string &admin_hash, char *status_message,
                                  const int status_sz)
{
	std::string message = "";
	MJR_DEBUG("Getting geom properties requested");
	if (!VerifyAdminHash(admin_hash)) {
		message = "Unauthorized geom properties request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	if (geom_name.empty()) {
		message = "Geom name is empty. Cannot get geom properties";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, geom_name.c_str());
	if (geom_id == -1) {
		message = "Could not find model (mujoco geom) with name " + geom_name;
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	int body_id = model_->geom_bodyid[geom_id];

	// Lock mutex to get data within one step
	std::lock_guard<std::recursive_mutex> lk_sim(physics_thread_mutex_);
	body_mass      = model_->body_mass[body_id];
	friction_slide = model_->geom_friction[geom_id * 3];
	friction_spin  = model_->geom_friction[geom_id * 3 + 1];
	friction_roll  = model_->geom_friction[geom_id * 3 + 2];
	size_x         = model_->geom_size[geom_id * 3];
	size_y         = model_->geom_size[geom_id * 3 + 1];
	size_z         = model_->geom_size[geom_id * 3 + 2];
	type           = model_->geom_type[geom_id];

	return true;
}

bool MujocoEnv::SetEqualityConstraintParameters(const std::string &eq_name, const int &type,
                                                const mjtNum *solver_params, const bool &active,
                                                const std::string &element1, const std::string &element2,
                                                const mjtNum &torquescale, const mjtNum *anchor, const mjtNum *relpose,
                                                const mjtNum *polycoef, const std::string &admin_hash,
                                                char *status_message, const int status_sz)
{
	std::string message = "";
	MJR_DEBUG_STREAM("Setting equality constraint '" << eq_name << "'");

	if (!VerifyAdminHash(admin_hash)) {
		message = "Unauthorized equality constraint request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}
	int eq_id = mj_name2id(model_.get(), mjOBJ_EQUALITY, eq_name.c_str());
	if (eq_id == -1) {
		message = "Could not find equality constraint named '" + eq_name + "'";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	mj_markStack(data_.get());
	int *ids = mj_stackAllocInt(data_.get(), 2);

	switch (type) {
		case mjEQ_TENDON:
			if (!element1.empty()) {
				MJR_DEBUG_STREAM("\tSetting element1 of tendon eqc to '" << element1 << "'");
				ids[0] = mj_name2id(model_.get(), mjOBJ_TENDON, element1.c_str());
				if (ids[0] != -1) {
					model_->eq_obj1id[eq_id] = ids[0];
				}
				MJR_WARN_STREAM_COND(ids[0] == -1, "Could not find tendon with name '" << element1 << "'");
			}
			if (!element2.empty()) {
				MJR_DEBUG_STREAM("\tSetting element2 of tendon eqc to '" << element2.c_str() << "'");
				ids[1] = mj_name2id(model_.get(), mjOBJ_TENDON, element2.c_str());
				if (ids[1] != -1) {
					model_->eq_obj2id[eq_id] = ids[1];
				}
				MJR_WARN_STREAM_COND(ids[1] == -1, "Could not find tendon with name '" << element2 << "'");
			}
			if (polycoef != nullptr) {
				MJR_DEBUG_STREAM("\tSetting polycoef of tendon eqc '" << eq_name << "' to " << polycoef[0] << ", "
				                                                      << polycoef[1] << ", " << polycoef[2] << ", "
				                                                      << polycoef[3] << ", " << polycoef[4]);
				mju_copy(model_->eq_data + eq_id * mjNEQDATA, polycoef, 5);
			}
			break;
		case mjEQ_WELD:
			if (!element1.empty()) {
				MJR_DEBUG_STREAM("\tSetting element1 of weld eqc to '" << element1 << "'");
				ids[0] = mj_name2id(model_.get(), mjOBJ_XBODY, element1.c_str());
				if (ids[0] != -1) {
					model_->eq_obj1id[eq_id] = ids[0];
				}
				MJR_WARN_STREAM_COND(ids[0] == -1, "Could not find body with name '" << element1 << "'");
			}
			if (!element2.empty()) {
				MJR_DEBUG_STREAM("\tSetting element2 of weld eqc to '" << element2 << "'");
				ids[1] = mj_name2id(model_.get(), mjOBJ_XBODY, element2.c_str());
				if (ids[1] != -1) {
					model_->eq_obj2id[eq_id] = ids[1];
				}
				MJR_WARN_STREAM_COND(ids[1] == -1, "Could not find body with name '" << element2 << "'");
			}
			if (anchor != nullptr) {
				MJR_DEBUG_STREAM("\tSetting anchor of weld eqc to " << anchor[0] << ", " << anchor[1] << ", " << anchor[2]);
				mju_copy(model_->eq_data + eq_id * mjNEQDATA, anchor, 3);
			}
			if (relpose != nullptr) {
				MJR_DEBUG_STREAM("\tSetting relpose of weld eqc to "
				                 << relpose[0] << ", " << relpose[1] << ", " << relpose[2] << ", " << relpose[3] << ", "
				                 << relpose[4] << ", " << relpose[5] << ", " << relpose[6] << ", " << relpose[7] << ", "
				                 << relpose[8] << ", " << relpose[9]);
				mju_copy(model_->eq_data + eq_id * mjNEQDATA + 3, relpose, 7);
			}
			if (torquescale != -1) {
				MJR_DEBUG_STREAM("\tSetting torquescale of weld eqc to " << torquescale);
				model_->eq_data[eq_id * mjNEQDATA + 10] = torquescale;
			}
			break;
		case mjEQ_JOINT:
			if (!element1.empty()) {
				MJR_DEBUG_STREAM("\tSetting element1 of joint eqc to '" << element1 << "'");
				ids[0] = mj_name2id(model_.get(), mjOBJ_JOINT, element1.c_str());
				if (ids[0] != -1) {
					model_->eq_obj1id[eq_id] = ids[0];
				}
				MJR_WARN_STREAM_COND(ids[0] == -1, "Could not find joint with name '" << element1 << "'");
			}
			if (!element2.empty()) {
				MJR_DEBUG_STREAM("\tSetting element2 of joint eqc to '" << element2 << "'");
				ids[1] = mj_name2id(model_.get(), mjOBJ_JOINT, element2.c_str());
				if (ids[1] != -1) {
					model_->eq_obj2id[eq_id] = ids[1];
				}
				MJR_WARN_STREAM_COND(ids[1] == -1, "Could not find joint with name '" << element2 << "'");
			}
			if (polycoef != nullptr) {
				MJR_DEBUG_STREAM("\tSetting polycoef of joint eqc '" << eq_name << "' to " << polycoef[0] << ", "
				                                                     << polycoef[1] << ", " << polycoef[2] << ", "
				                                                     << polycoef[3] << ", " << polycoef[4]);
				mju_copy(model_->eq_data + eq_id * mjNEQDATA, polycoef, 5);
			}
			break;
		case mjEQ_CONNECT:
			if (!element1.empty()) {
				MJR_DEBUG_STREAM("\tSetting element1 of connect eqc to '" << element1 << "'");
				ids[0] = mj_name2id(model_.get(), mjOBJ_XBODY, element1.c_str());
				if (ids[0] != -1) {
					model_->eq_obj1id[eq_id] = ids[0];
				}
				MJR_WARN_STREAM_COND(ids[0] == -1, "Could not find body with name '" << element1 << "'");
			}
			if (!element2.empty()) {
				MJR_DEBUG_STREAM("\tSetting element2 of connect eqc to '" << element2 << "'");
				ids[1] = mj_name2id(model_.get(), mjOBJ_XBODY, element2.c_str());
				if (ids[1] != -1) {
					model_->eq_obj2id[eq_id] = ids[1];
				}
				MJR_WARN_STREAM_COND(ids[1] == -1, "Could not find body with name '" << element2 << "'");
			}
			if (anchor != nullptr) {
				MJR_DEBUG_STREAM("\tSetting anchor of connect eqc to " << anchor[0] << ", " << anchor[1] << ", "
				                                                       << anchor[2]);
				mju_copy(model_->eq_data + eq_id * mjNEQDATA, anchor, 3);
			}
			break;

		default:
			MJR_ERROR_STREAM("Invalid equality constraint type '" << type << "'"
			                                                      << ". Must be a valid mjEQ type");
			break;
	}
	mj_freeStack(data_.get());
	data_->eq_active[eq_id] = active;
	if (solver_params != nullptr) {
		MJR_DEBUG_STREAM("\tSetting solver parameters of eqc to " << solver_params[0] << ", " << solver_params[1] << ", "
		                                                          << solver_params[2] << ", " << solver_params[3] << ", "
		                                                          << solver_params[4] << ", " << solver_params[5] << ", "
		                                                          << solver_params[6]);
		mju_copy(model_->eq_solimp + eq_id * mjNIMP, solver_params, mjNIMP);
		mju_copy(model_->eq_solref + eq_id * mjNREF, solver_params + mjNIMP, mjNREF);
	}
	return true;
}

bool MujocoEnv::GetEqualityConstraintParameters(const std::string &eq_name, int &type, mjtNum *solver_params,
                                                bool &active, std::string &element1, std::string &element2,
                                                mjtNum &torquescale, mjtNum *anchor, mjtNum *relpose, mjtNum *polycoef,
                                                const std::string &admin_hash, char *status_message,
                                                const int status_sz)
{
	MJR_DEBUG_STREAM("Getting equality constraint '" << eq_name << "'");
	if (!VerifyAdminHash(admin_hash)) {
		std::string message = "Unauthorized equality constraint request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}
	int eq_id = mj_name2id(model_.get(), mjOBJ_EQUALITY, eq_name.c_str());
	if (eq_id == -1) {
		std::string message = "Could not find equality constraint named '" + eq_name + "'";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}
	MJR_DEBUG_STREAM("Found equality constraint '" << eq_name << "' with id " << eq_id);

	type   = model_->eq_type[eq_id];
	active = data_->eq_active[eq_id];
	mju_copy(solver_params, model_->eq_solimp + eq_id * mjNIMP, mjNIMP);
	mju_copy(solver_params + mjNIMP, model_->eq_solref + eq_id * mjNREF, mjNREF);

	switch (type) {
		case mjEQ_TENDON: {
			const char *e1_c = mj_id2name(model_.get(), mjOBJ_TENDON, model_->eq_obj1id[eq_id]);
			const char *e2_c = mj_id2name(model_.get(), mjOBJ_TENDON, model_->eq_obj2id[eq_id]);
			if (e1_c != nullptr) {
				element1 = e1_c;
			}
			if (e2_c != nullptr) {
				element2 = e2_c;
			}
			mju_copy(polycoef, model_->eq_data + eq_id * mjNEQDATA, 5);
			break;
		}
		case mjEQ_WELD: {
			const char *e1_c = mj_id2name(model_.get(), mjOBJ_XBODY, model_->eq_obj1id[eq_id]);
			const char *e2_c = mj_id2name(model_.get(), mjOBJ_XBODY, model_->eq_obj2id[eq_id]);
			if (e1_c != nullptr) {
				element1 = e1_c;
			}
			if (e2_c != nullptr) {
				element2 = e2_c;
			}
			mju_copy(anchor, model_->eq_data + eq_id * mjNEQDATA, 3);
			mju_copy(relpose, model_->eq_data + eq_id * mjNEQDATA + 3, 10);
			torquescale = model_->eq_data[eq_id * mjNEQDATA + 10];
			break;
		}
		case mjEQ_JOINT: {
			const char *e1_c = mj_id2name(model_.get(), mjOBJ_JOINT, model_->eq_obj1id[eq_id]);
			const char *e2_c = mj_id2name(model_.get(), mjOBJ_JOINT, model_->eq_obj2id[eq_id]);
			if (e1_c != nullptr) {
				element1 = e1_c;
			}
			if (e2_c != nullptr) {
				element2 = e2_c;
			}
			mju_copy(polycoef, model_->eq_data + eq_id * mjNEQDATA, 5);
			break;
		}
		case mjEQ_CONNECT: {
			const char *e1_c = mj_id2name(model_.get(), mjOBJ_XBODY, model_->eq_obj1id[eq_id]);
			const char *e2_c = mj_id2name(model_.get(), mjOBJ_XBODY, model_->eq_obj2id[eq_id]);
			if (e1_c != nullptr) {
				element1 = e1_c;
			}
			if (e2_c != nullptr) {
				element2 = e2_c;
			}
			mju_copy(anchor, model_->eq_data + eq_id * mjNEQDATA, 3);
			break;
		}
		default:
			MJR_ERROR_STREAM("Invalid equality constraint type '" << type << "'"
			                                                      << ". Must be a valid mjEQ type");
			break;
	}
	return true;
}

void MujocoEnv::GetSimulationStatus(int &status, std::string &description)
{
	status = GetOperationalStatus();

	switch (status) {
		case 0:
			description = "Simulation is ready";
			break;
		case 1:
			description = "Loading in progress";
			break;
		default: // >= 2
			description = "Loading issued";
			break;
	}
}

void MujocoEnv::GetSimInfo(std::string &model_path, bool &model_valid, int &load_count, int &loading_state,
                           std::string &loading_description, bool &paused, int &pending_sim_steps, float &rt_measured,
                           float &rt_setting)
{
	model_path  = filename_;
	model_valid = sim_state_.model_valid;
	load_count  = sim_state_.load_count;
	GetSimulationStatus(loading_state, loading_description);
	paused            = !settings_.run.load();
	pending_sim_steps = settings_.env_steps_request.load();
	rt_measured       = 1.f / sim_state_.measured_slowdown;
	rt_setting        = percentRealTime[settings_.real_time_index] / 100.f;
}

// Helper function to retrieve the real-time factor closest to the requested value
// adapted from https://www.geeksforgeeks.org/find-closest-number-array/
float FindClosestRecursive(const float arr[], uint left, uint right, float target)
{
	if (left == right) {
		return arr[left];
	}

	uint mid            = (left + right) / 2;
	float left_closest  = FindClosestRecursive(arr, left, mid, target);
	float right_closest = FindClosestRecursive(arr, mid + 1, right, target);

	if (abs(left_closest - target) <= abs(right_closest - target)) {
		return left_closest;
	} else {
		return right_closest;
	}
}

bool MujocoEnv::SetRealTimeFactor(const float &rt_factor, const std::string &admin_hash, char *status_message,
                                  const int status_sz)
{
	MJR_DEBUG_STREAM("Setting real-time factor to " << rt_factor);
	if (!VerifyAdminHash(admin_hash)) {
		std::string message = "Unauthorized real-time factor request detected. Ignoring request";
		MJR_ERROR_STREAM(message);
		if (status_message != nullptr) {
			std::strncpy(status_message, message.c_str(), status_sz);
		}
		return false;
	}

	if (rt_factor < 0) {
		MJR_DEBUG("Setting to unbound rt mode");
		settings_.real_time_index = 0;
		settings_.speed_changed   = true;
		return true;
	}

	size_t num_clicks = sizeof(percentRealTime) / sizeof(percentRealTime[0]);
	// start at 1 to skip the unbound mode if the rt_factor value is small but non-negative.
	float closest_rt = FindClosestRecursive(percentRealTime, 1, num_clicks - 1, rt_factor * 100.f);
	MJR_WARN_STREAM_COND(fabs(closest_rt / 100.f - rt_factor) > 0.001f,
	                     "Requested real-time factor " << rt_factor
	                                                   << " is not available. Setting to closest available value "
	                                                   << closest_rt / 100.f);

	// get index of closest real-time factor
	auto it                   = std::find(std::begin(percentRealTime), std::end(percentRealTime), closest_rt);
	settings_.real_time_index = std::distance(std::begin(percentRealTime), it);
	settings_.speed_changed   = true;
	return true;
}

int MujocoEnv::GetPluginStats(std::vector<std::string> &names, std::vector<std::string> &types,
                              std::vector<double> &load_time, std::vector<double> &reset_time,
                              std::vector<double> &step_time_control, std::vector<double> &step_time_passive,
                              std::vector<double> &step_time_render, std::vector<double> &step_time_last_stage)
{
	names.clear();
	types.clear();
	load_time.clear();
	reset_time.clear();
	step_time_control.clear();
	step_time_passive.clear();
	step_time_render.clear();
	step_time_last_stage.clear();

	RecursiveLock sim_lock(physics_thread_mutex_);
	for (const auto &plugin : plugins_) {
		names.emplace_back(plugin->get_name());
		types.emplace_back(plugin->get_type());
		load_time.emplace_back(plugin->get_load_time());
		reset_time.emplace_back(plugin->get_reset_time());
		step_time_control.emplace_back(plugin->get_ema_steptime_control());
		step_time_passive.emplace_back(plugin->get_ema_steptime_passive());
		step_time_render.emplace_back(plugin->get_ema_steptime_render());
		step_time_last_stage.emplace_back(plugin->get_ema_steptime_last_stage());
	}
	return plugins_.size();
}

} // namespace mujoco_ros
