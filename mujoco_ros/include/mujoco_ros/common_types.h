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

#include <mujoco/mujoco.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <GLFW/glfw3.h>

static constexpr int kBufSize = 1000;

namespace MujocoSim {

namespace rendering {

typedef enum streamType_ : uint8_t
{
	RGB       = 1,
	DEPTH     = 1 << 1,
	SEGMENTED = 1 << 1 << 1,

	// Combined types to be cast safe
	RGB_D   = 3,
	RGB_S   = 5,
	DEPTH_S = 6,
	RGB_D_S = 7
} streamType;

struct VisualStruct
{
	mjvScene scn                           = {};
	mjvCamera cam                          = {};
	mjvOption vopt                         = {};
	mjrContext con                         = {};
	boost::shared_ptr<unsigned char[]> rgb = {};
	boost::shared_ptr<float[]> depth       = {};
	GLFWwindow *window                     = nullptr;

	~VisualStruct()
	{
		if (window != nullptr) {
			glfwMakeContextCurrent(window);
			ROS_DEBUG("Freeing scene in vis");
			mjv_freeScene(&scn);
			ROS_DEBUG("Freeing context in vis");
			mjr_freeContext(&con);
			glfwDestroyWindow(window);
		}
	};
};

class CameraStream;
typedef boost::shared_ptr<CameraStream> CameraStreamPtr;

} // namespace rendering

/**
 * @def mjModelPtr
 * @brief boost::shared_ptr to mjModel
 */
typedef boost::shared_ptr<mjModel> mjModelPtr;
/**
 * @def mjDataPtr
 * @brief boost::shared_ptr to mjData
 */
typedef boost::shared_ptr<mjData> mjDataPtr;

// MujocoPlugin
class MujocoPlugin;

/**
 * @def MujocoPluginPtr
 * @brief boost::shared_ptr to MujocoPlugin
 */
typedef boost::shared_ptr<MujocoPlugin> MujocoPluginPtr;

// MujocoEnvironment
struct MujocoEnv;

/**
 * @def MujocoEnvPtr
 * @brief boost::shared_ptr to MujocoEnv
 */
typedef boost::shared_ptr<MujocoEnv> MujocoEnvPtr;

} // namespace MujocoSim
