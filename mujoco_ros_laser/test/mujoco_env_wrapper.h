/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <mujoco_ros/mujoco_env.hpp>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string()) : MujocoEnv(admin_hash) {}
	mjModel *getModelPtr() { return model_.get(); }
	mjData *getDataPtr() { return data_.get(); }
	MujocoEnvMutex *getMutexPtr() { return &physics_thread_mutex_; }
	int getPendingSteps() { return num_steps_until_exit_; }

	void setEvalMode(bool eval_mode) { settings_.eval_mode = eval_mode; }
	void setAdminHash(const std::string &hash) { mju::strcpy_arr(settings_.admin_hash, hash.c_str()); }

	std::string getFilename() { return { filename_ }; }
	int isPhysicsRunning() { return is_physics_running_; }
	int isEventRunning() { return is_event_running_; }
	int isRenderingRunning() { return is_rendering_running_; }

	int getNumCBReadyPlugins() { return cb_ready_plugins_.size(); }
	void notifyGeomChange() { notifyGeomChanged(0); }

	void load_filename(const std::string &filename)
	{
		mju::strcpy_arr(queued_filename_, filename.c_str());
		settings_.load_request = 2;
	}

	void shutdown()
	{
		settings_.exit_request = 1;
		waitForPhysicsJoin();
		waitForEventsJoin();
	}

	const std::string &getHandleNamespace() { return nh_->getNamespace(); }

	void startWithXML(const std::string &xml_path, bool wait = false)
	{
		mju::strcpy_arr(queued_filename_, xml_path.c_str());
		settings_.load_request = 2;
		startPhysicsLoop();
		startEventLoop();
	}
};
