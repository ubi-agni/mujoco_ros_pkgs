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
#include <mujoco_ros/mujoco_env.h>

namespace mujoco_ros::python {
using namespace mujoco_ros;

void InitEnvSettingsPy(py::module &m)
{
	py::class_<SimState>(m, "_SimState")
	    .def_readonly("measured_slowdown", &SimState::measured_slowdown)
	    .def_readonly("model_valid", &SimState::model_valid)
	    .def_readonly("load_count", &SimState::load_count)
	    .def("__repr__", [](const SimState &d) {
		    return std::string("<SimState\n") + "  measured_slowdown: " + std::to_string(d.measured_slowdown) + "\n" +
		           "  model_valid: " + std::to_string(d.model_valid) + "\n" +
		           "  load_count: " + std::to_string(d.load_count) + "\n" + ">";
	    });

	py::class_<EnvSettings>(m, "_EnvSettings")
	    .def_readonly("headless", &EnvSettings::headless,
	                  "If true, the environment is running in headless mode without any rendering.")
	    .def_readonly("render_offscreen", &EnvSettings::render_offscreen,
	                  "If true, the environment is rendering offscreen.")
	    .def_readonly("use_sim_time", &EnvSettings::use_sim_time,
	                  "If true, the environment uses simulation time instead of wall clock time.")
	    .def_property(
	        "real_time_index", [](EnvSettings &self) { return self.real_time_index; },
	        [](EnvSettings &self) {
		        // TODO: Once ROS2 branch is merged use interface function implementing thread safety
		        printf("Setting real_time_index not implemented in Python bindings");
	        },
	        "The index of the real-time factor in the simulation.")
	    .def_readwrite(
	        "busywait", &EnvSettings::busywait,
	        "If true, the environment is using busy-waiting for running steps with the desired real-time factor.")
	    .def_readonly("eval_mode", &EnvSettings::eval_mode, "If true, the environment is running in evaluation mode.")
	    .def_property(
	        "run", [](EnvSettings &self) { return self.run.load(); },
	        [](EnvSettings &self, bool run) { self.run.store(run); })
	    .def_property(
	        "exit_request", [](EnvSettings &self) { return self.exit_request.load(); },
	        [](EnvSettings &self, int value) { self.exit_request.store(value); })
	    .def_property(
	        "visual_init_request", [](EnvSettings &self) { return self.visual_init_request.load(); },
	        [](EnvSettings &self, int visual_init_request) { self.visual_init_request.store(visual_init_request); })
	    .def_property(
	        "reset_request", [](EnvSettings &self) { return self.reset_request.load(); },
	        [](EnvSettings &self, int reset_request) { self.reset_request.store(reset_request); })
	    .def_property(
	        "env_steps_request", [](EnvSettings &self) { return self.env_steps_request.load(); },
	        [](EnvSettings &self, int env_steps_request) { self.env_steps_request.store(env_steps_request); })
	    .def("__repr__", [](const mujoco_ros::EnvSettings &d) {
		    return std::string("<EnvSettings\n") + "  headless: " + std::to_string(d.headless) + "\n" +
		           "  render_offscreen: " + std::to_string(d.render_offscreen) + "\n" +
		           "  use_sim_time: " + std::to_string(d.use_sim_time) + "\n" +
		           "  real_time_index: " + std::to_string(d.real_time_index) + "\n" +
		           "  busywait: " + std::to_string(d.busywait) + "\n" + "  eval_mode: " + std::to_string(d.eval_mode) +
		           "\n" + "  run: " + std::to_string(d.run.load()) + "\n" +
		           "  exit_request: " + std::to_string(d.reset_request.load()) + "\n" +
		           "  reset_request: " + std::to_string(d.reset_request.load()) + "\n" +
		           "  env_steps_request: " + std::to_string(d.env_steps_request.load()) + "\n" + ">";
	    });
}

} // namespace mujoco_ros::python
