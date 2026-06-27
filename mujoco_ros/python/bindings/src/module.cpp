/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
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

#include "pymujoco_ros.hpp"

#include <mujoco/mujoco.h>
#include <mujoco_ros/render_backend.hpp>

namespace {

#if RENDER_BACKEND == GLFW_BACKEND
constexpr const char *kRenderBackend = "GLFW";
#elif RENDER_BACKEND == OSMESA_BACKEND
constexpr const char *kRenderBackend = "OSMesa";
#elif RENDER_BACKEND == EGL_BACKEND
constexpr const char *kRenderBackend = "EGL";
#else
constexpr const char *kRenderBackend = "NONE";
#endif

} // namespace

namespace mujoco_ros::python {

PYBIND11_MODULE(pymujoco_ros, module)
{
	module.attr("__mujoco_version__") = mj_versionString();
	module.attr("__render_backend__") = kRenderBackend;

	InitEnvStructs(module);
	InitPlugins(module);
	InitRendering(module);
	InitMujocoEnv(module);
}

} // namespace mujoco_ros::python
