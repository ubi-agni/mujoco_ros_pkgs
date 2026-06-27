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

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#else
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#endif

namespace mujoco_ros::python {
namespace {

struct EnginePluginLoader
{
	EnginePluginLoader() { plugin_utils::InitPluginLoader(); }
};

EnginePluginLoader engine_plugin_loader;

} // namespace

void InitPlugins(py::module_ &module)
{
	py::class_<MujocoPlugin, std::shared_ptr<MujocoPlugin>>(module, "_MujocoPlugin")
	    .def_property_readonly("name", &MujocoPlugin::get_name)
	    .def_property_readonly("type", &MujocoPlugin::get_type)
	    .def_property_readonly("is_loaded", &MujocoPlugin::is_loaded)
	    .def_property_readonly("load_time", &MujocoPlugin::get_load_time)
	    .def_property_readonly("reset_time", &MujocoPlugin::get_reset_time)
	    .def_property_readonly("ema_steptime_control", &MujocoPlugin::get_ema_steptime_control)
	    .def_property_readonly("ema_steptime_passive", &MujocoPlugin::get_ema_steptime_passive)
	    .def_property_readonly("ema_steptime_render", &MujocoPlugin::get_ema_steptime_render)
	    .def_property_readonly("ema_steptime_last_stage", &MujocoPlugin::get_ema_steptime_last_stage)
	    .def("__repr__", [](const MujocoPlugin &plugin) {
		    return "<MujocoPlugin name='" + plugin.get_name() + "' type='" + plugin.get_type() + "'>";
	    });
}

} // namespace mujoco_ros::python
