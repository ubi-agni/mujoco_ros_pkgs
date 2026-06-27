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

#include <memory>

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros_control/mujoco_ros_control_plugin.hpp>
#else
#include <mujoco_ros_control/ros_two/mujoco_ros_control.hpp>
#endif

namespace py = pybind11;

namespace mujoco_ros::python::control {
namespace {

mujoco_ros::control::MujocoRosControlPlugin *Bind(mujoco_ros::MujocoPlugin *plugin)
{
	auto *typed_plugin = dynamic_cast<mujoco_ros::control::MujocoRosControlPlugin *>(plugin);
	if (typed_plugin == nullptr) {
		throw py::type_error("plugin is not a MujocoRosControlPlugin");
	}
	return typed_plugin;
}

} // namespace

PYBIND11_MODULE(pymujoco_ros_control, module)
{
	py::module_::import("pymujoco_ros");

	auto cls =
	    py::class_<mujoco_ros::control::MujocoRosControlPlugin, mujoco_ros::MujocoPlugin,
	               std::shared_ptr<mujoco_ros::control::MujocoRosControlPlugin>>(module, "MujocoRosControlPlugin");

	cls.def_property_readonly("robot_description_param",
	                          &mujoco_ros::control::MujocoRosControlPlugin::GetRobotDescriptionParam)
	    .def_property_readonly("control_period", &mujoco_ros::control::MujocoRosControlPlugin::GetControlPeriodSec)
	    .def_property_readonly("has_controller_manager",
	                           &mujoco_ros::control::MujocoRosControlPlugin::HasControllerManager)
	    .def("__repr__", [](const mujoco_ros::control::MujocoRosControlPlugin &plugin) {
		    return "<MujocoRosControlPlugin name='" + plugin.get_name() + "' type='" + plugin.get_type() + "'>";
	    });

#if MJR_ROS_VERSION == ROS_1
	cls.def_property_readonly("robot_namespace", &mujoco_ros::control::MujocoRosControlPlugin::GetRobotNamespace)
	    .def_property_readonly("robot_hw_sim_type", &mujoco_ros::control::MujocoRosControlPlugin::GetRobotHWSimType)
	    .def_property_readonly("transmission_count", &mujoco_ros::control::MujocoRosControlPlugin::GetTransmissionCount)
	    .def_property_readonly("has_robot_hw_sim", &mujoco_ros::control::MujocoRosControlPlugin::HasRobotHWSim)
	    .def_property_readonly("e_stop_active", &mujoco_ros::control::MujocoRosControlPlugin::IsEStopActive);
#else
	cls.def_property_readonly("robot_description_node",
	                          &mujoco_ros::control::MujocoRosControlPlugin::GetRobotDescriptionNode)
	    .def_property_readonly("update_rate", &mujoco_ros::control::MujocoRosControlPlugin::GetUpdateRate);
#endif

	module.def("bind", &Bind, py::arg("plugin"), py::return_value_policy::reference);
}

} // namespace mujoco_ros::python::control
