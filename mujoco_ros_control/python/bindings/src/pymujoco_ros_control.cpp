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
#include <pybind11/stl.h>

#include <memory>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/mujoco_env.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros_control/mujoco_ros_control_plugin.hpp>
#else
#include <mujoco_ros_control/ros_two/mujoco_ros_control.hpp>
#endif

namespace py = pybind11;

namespace mujoco_ros::python::control {
namespace {

struct ControlPluginHandle
{
	explicit ControlPluginHandle(mujoco_ros::PluginHandle handle) : handle(std::move(handle)) {}
	mujoco_ros::PluginHandle handle;
};

std::shared_ptr<ControlPluginHandle> Bind(py::object plugin)
{
	auto handle = py::cast<mujoco_ros::PluginHandle>(plugin);
	if (handle.Type() != "mujoco_ros_control/MujocoRosControlPlugin") {
		throw py::type_error("plugin is not a MujocoRosControlPlugin");
	}
	return std::make_shared<ControlPluginHandle>(std::move(handle));
}

} // namespace

PYBIND11_MODULE(pymujoco_ros_control, module)
{
	py::module_::import("pymujoco_ros");

	py::class_<ControlPluginHandle, std::shared_ptr<ControlPluginHandle>> cls(module, "MujocoRosControlPlugin");

	cls.def_property_readonly("robot_description_param",
	                          [](const ControlPluginHandle &handle) {
		                          return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                              [](const auto &plugin) { return std::string(plugin.GetRobotDescriptionParam()); });
	                          })
	    .def_property_readonly("control_period",
	                           [](const ControlPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                               [](const auto &plugin) { return plugin.GetControlPeriodSec(); });
	                           })
	    .def_property_readonly("has_controller_manager",
	                           [](const ControlPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                               [](const auto &plugin) { return plugin.HasControllerManager(); });
	                           })
	    .def("__repr__", [](const ControlPluginHandle &plugin) {
		    return "<MujocoRosControlPlugin name='" + plugin.handle.Name() + "' type='" + plugin.handle.Type() + "'>";
	    });

#if MJR_ROS_VERSION == ROS_1
	cls.def_property_readonly("robot_namespace",
	                          [](const ControlPluginHandle &handle) {
		                          return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                              [](const auto &plugin) { return std::string(plugin.GetRobotNamespace()); });
	                          })
	    .def_property_readonly("robot_hw_sim_type",
	                           [](const ControlPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                               [](const auto &plugin) { return std::string(plugin.GetRobotHWSimType()); });
	                           })
	    .def_property_readonly("transmission_count",
	                           [](const ControlPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                               [](const auto &plugin) { return plugin.GetTransmissionCount(); });
	                           })
	    .def_property_readonly("has_robot_hw_sim",
	                           [](const ControlPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                               [](const auto &plugin) { return plugin.HasRobotHWSim(); });
	                           })
	    .def_property_readonly("e_stop_active", [](const ControlPluginHandle &handle) {
		    return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		        [](const auto &plugin) { return plugin.IsEStopActive(); });
	    });
#else
	cls.def_property_readonly("robot_description_node",
	                          [](const ControlPluginHandle &handle) {
		                          return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		                              [](const auto &plugin) { return std::string(plugin.GetRobotDescriptionNode()); });
	                          })
	    .def_property_readonly("update_rate", [](const ControlPluginHandle &handle) {
		    return handle.handle.WithBackend<mujoco_ros::control::MujocoRosControlPlugin>(
		        [](const auto &plugin) { return plugin.GetUpdateRate(); });
	    });
#endif

	module.def("bind", &Bind, py::arg("plugin"));
}

} // namespace mujoco_ros::python::control
