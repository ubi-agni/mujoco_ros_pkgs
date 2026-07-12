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

#include <mujoco_ros/mujoco_env.hpp>

#include <string>

namespace mujoco_ros::python {
namespace {

std::string BoolString(bool value)
{
	return value ? "true" : "false";
}

} // namespace

void InitEnvStructs(py::module_ &module)
{
	py::class_<EnvSettings>(module, "_EnvSettings")
	    .def_property_readonly("headless", [](const EnvSettings &settings) { return settings.headless; })
	    .def_property_readonly("render_offscreen", [](const EnvSettings &settings) { return settings.render_offscreen; })
	    .def_property_readonly("use_sim_time", [](const EnvSettings &settings) { return settings.use_sim_time; })
	    .def_property_readonly(
	        "real_time_index", [](const EnvSettings &settings) { return settings.real_time_index; },
	        "The index of the real-time factor in the simulation.")
	    .def_readwrite(
	        "busywait", &EnvSettings::busywait,
	        "If true, the environment is using busy-waiting for running steps with the desired real-time factor.")
	    .def_property_readonly("num_mj_threads", [](const EnvSettings &settings) { return settings.num_mj_threads; })
	    .def_property_readonly("eval_mode", [](const EnvSettings &settings) { return settings.eval_mode; })
	    .def_property_readonly("admin_hash",
	                           [](const EnvSettings &settings) { return std::string(settings.admin_hash); })
	    .def_property_readonly("run", [](const EnvSettings &settings) { return settings.run.load(); })
	    .def_property_readonly("exit_request", [](const EnvSettings &settings) { return settings.exit_request.load(); })
	    .def_property_readonly("visual_init_request",
	                           [](const EnvSettings &settings) { return settings.visual_init_request.load(); })
	    .def_property_readonly("load_request", [](const EnvSettings &settings) { return settings.load_request.load(); })
	    .def_property_readonly("reset_request",
	                           [](const EnvSettings &settings) { return settings.reset_request.load(); })
	    .def_property_readonly("speed_changed",
	                           [](const EnvSettings &settings) { return settings.speed_changed.load(); })
	    .def_property_readonly("env_steps_request",
	                           [](const EnvSettings &settings) { return settings.env_steps_request.load(); })
	    .def_property_readonly("settings_changed",
	                           [](const EnvSettings &settings) { return settings.settings_changed.load(); })
	    .def_property_readonly("is_python_request",
	                           [](const EnvSettings &settings) { return settings.is_python_request.load(); })
	    .def("__repr__", [](const EnvSettings &settings) {
		    return "<EnvSettings headless=" + BoolString(settings.headless) +
		           " render_offscreen=" + BoolString(settings.render_offscreen) +
		           " use_sim_time=" + BoolString(settings.use_sim_time) +
		           " real_time_index=" + std::to_string(settings.real_time_index) +
		           " run=" + std::to_string(settings.run.load()) + ">";
	    });

	py::class_<SimState>(module, "_SimState")
	    .def_readonly("measured_slowdown", &SimState::measured_slowdown)
	    .def_readonly("model_valid", &SimState::model_valid)
	    .def_readonly("load_count", &SimState::load_count)
	    .def("__repr__", [](const SimState &state) {
		    return "<SimState measured_slowdown=" + std::to_string(state.measured_slowdown) +
		           " model_valid=" + BoolString(state.model_valid) + " load_count=" + std::to_string(state.load_count) +
		           ">";
	    });

	py::class_<SimInfo>(module, "_SimInfo")
	    .def_readonly("model_path", &SimInfo::model_path)
	    .def_readonly("model_valid", &SimInfo::model_valid)
	    .def_readonly("load_count", &SimInfo::load_count)
	    .def_readonly("loading_state", &SimInfo::loading_state)
	    .def_readonly("loading_description", &SimInfo::loading_description)
	    .def_readonly("paused", &SimInfo::paused)
	    .def_readonly("pending_sim_steps", &SimInfo::pending_sim_steps)
	    .def_readonly("rt_measured", &SimInfo::rt_measured)
	    .def_readonly("rt_setting", &SimInfo::rt_setting)
	    .def("__repr__", [](const SimInfo &info) {
		    return "<SimInfo model_path='" + info.model_path + "' model_valid=" + BoolString(info.model_valid) +
		           " load_count=" + std::to_string(info.load_count) + " paused=" + BoolString(info.paused) + ">";
	    });

	py::class_<PluginStat>(module, "_PluginStat")
	    .def_readonly("name", &PluginStat::name)
	    .def_readonly("type", &PluginStat::type)
	    .def_readonly("load_time", &PluginStat::load_time)
	    .def_readonly("reset_time", &PluginStat::reset_time)
	    .def_readonly("ema_steptime_control", &PluginStat::ema_steptime_control)
	    .def_readonly("ema_steptime_passive", &PluginStat::ema_steptime_passive)
	    .def_readonly("ema_steptime_render", &PluginStat::ema_steptime_render)
	    .def_readonly("ema_steptime_last_stage", &PluginStat::ema_steptime_last_stage)
	    .def("__repr__",
	         [](const PluginStat &stat) { return "<PluginStat name='" + stat.name + "' type='" + stat.type + "'>"; });

	py::class_<RosAPISettings>(module, "_RosAPISettings")
	    .def_readonly("running", &RosAPISettings::running)
	    .def_readonly("admin_hash", &RosAPISettings::admin_hash)
	    .def("__repr__", [](const RosAPISettings &settings) {
		    return "<RosAPISettings running=" + BoolString(settings.running) + ">";
	    });
}

} // namespace mujoco_ros::python
