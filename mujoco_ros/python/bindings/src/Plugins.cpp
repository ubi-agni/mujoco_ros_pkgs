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

/* Authors: David P. Leins*/

#include "pymujoco_ros.h"
#include <mujoco_ros/plugin_utils.h>

namespace py = pybind11;
namespace mujoco_ros::python {
using namespace mujoco_ros;

namespace {
struct EnginePluginLoader
{
	EnginePluginLoader()
	{
		// This will register the MuJoCo engine plugins
		mujoco_ros::plugin_utils::registerEnginePlugins(true);
	}
};
} // namespace

static EnginePluginLoader _engine_plugin_loader;

void InitPluginsPy(py::module &m)
{
	py::class_<MujocoPlugin, std::shared_ptr<MujocoPlugin>>(m, "_MujocoPlugin")
	    .def_readonly("type", &MujocoPlugin::type_)
	    .def_readonly("load_time", &MujocoPlugin::load_time_)
	    .def_readonly("reset_time", &MujocoPlugin::reset_time_)
	    .def_readonly("ema_steptime_control", &MujocoPlugin::ema_steptime_control_)
	    .def_readonly("ema_steptime_passive", &MujocoPlugin::ema_steptime_passive_)
	    .def_readonly("ema_steptime_render", &MujocoPlugin::ema_steptime_render_)
	    .def_readonly("ema_steptime_last_stage", &MujocoPlugin::ema_steptime_last_stage_)
	    .def("__repr__",
	         [](const MujocoPlugin &d) { return std::string("<MujocoPlugin ") + "  type: " + d.type_ + " >"; });
}

} // namespace mujoco_ros::python
