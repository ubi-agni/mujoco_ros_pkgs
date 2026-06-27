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

/* Authors: David P. Leins */

#include <pybind11/pybind11.h>
#include <py_binding_tools/ros_msg_typecasters.h>

#include <memory>

#include <mujoco_ros_mocap/mocap_plugin.hpp>

namespace py = pybind11;

namespace mujoco_ros::python::mocap {
namespace {

mujoco_ros::mocap::MocapPlugin *Bind(mujoco_ros::MujocoPlugin *plugin)
{
	auto *typed_plugin = dynamic_cast<mujoco_ros::mocap::MocapPlugin *>(plugin);
	if (typed_plugin == nullptr) {
		throw py::type_error("plugin is not a MocapPlugin");
	}
	return typed_plugin;
}

void SetLastMocapState(mujoco_ros::mocap::MocapPlugin &plugin, const mujoco_ros::mocap::MocapState &state)
{
	if (!plugin.SetLastMocapState(state)) {
		throw py::value_error("invalid MocapState");
	}
}

} // namespace

PYBIND11_MODULE(pymujoco_ros_mocap, module)
{
	py::module_::import("pymujoco_ros");

	py::class_<mujoco_ros::mocap::MocapPlugin, mujoco_ros::MujocoPlugin,
	           std::shared_ptr<mujoco_ros::mocap::MocapPlugin>>(module, "MocapPlugin")
	    .def("get_current_mocaps_as_msg", &mujoco_ros::mocap::MocapPlugin::GetCurrentMocapsAsMsg)
	    .def_property(
	        "mocap_state", [](const mujoco_ros::mocap::MocapPlugin &plugin) { return plugin.GetLastMocapState(); },
	        &SetLastMocapState)
	    .def("__repr__", [](const mujoco_ros::mocap::MocapPlugin &plugin) {
		    return "<MocapPlugin name='" + plugin.get_name() + "' type='" + plugin.get_type() + "'>";
	    });

	module.def("bind", &Bind, py::arg("plugin"), py::return_value_policy::reference);
}

} // namespace mujoco_ros::python::mocap
