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

namespace mujoco_ros {

void MujocoEnv::StartPhysicsLoop()
{
	MJR_DEBUG("Starting physics loop");
	physics_thread_handle_ = std::thread(std::bind(&MujocoEnv::PhysicsLoop, this));
}

void MujocoEnv::WaitForPhysicsJoin()
{
	MJR_DEBUG("Waiting for physics join");
	if (physics_thread_handle_.joinable()) {
		physics_thread_handle_.join();
	}
	MJR_DEBUG("Physics joined");
}

void MujocoEnv::StartEventLoop()
{
	MJR_DEBUG("Starting event loop");
	event_thread_handle_ = std::thread(std::bind(&MujocoEnv::EventLoop, this));
}

void MujocoEnv::WaitForEventsJoin()
{
	MJR_DEBUG("Waiting for event join");
	if (event_thread_handle_.joinable()) {
		event_thread_handle_.join();
	}
	MJR_DEBUG("Event joined");
}

} // namespace mujoco_ros
