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

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros_laser/laser.hpp>

namespace py = pybind11;

namespace mujoco_ros::python::laser {
namespace {

struct LaserPluginHandle
{
	explicit LaserPluginHandle(mujoco_ros::PluginHandle handle) : handle(std::move(handle)) {}
	mujoco_ros::PluginHandle handle;
};

std::shared_ptr<LaserPluginHandle> Bind(py::object plugin)
{
	auto handle = py::cast<mujoco_ros::PluginHandle>(plugin);
	if (handle.Type() != "mujoco_ros_laser/LaserPlugin") {
		throw py::type_error("plugin is not a LaserPlugin");
	}
	return std::make_shared<LaserPluginHandle>(std::move(handle));
}

} // namespace

PYBIND11_MODULE(pymujoco_ros_laser, module)
{
	py::module_::import("pymujoco_ros");
	py::module_::import("pymujoco_ros_sensors");

	py::class_<mujoco_ros::sensors::laser::LaserConfig, mujoco_ros::sensors::SensorConfig>(module, "LaserConfig")
	    .def_property_readonly("name", [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.name; })
	    .def_property_readonly(
	        "site_attached", [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.site_attached; })
	    .def_property_readonly("visualize",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.visualize; })
	    .def_property_readonly("update_rate",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.update_rate; })
	    .def_property_readonly("min_range",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.min_range; })
	    .def_property_readonly("max_range",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.max_range; })
	    .def_property_readonly(
	        "range_resolution",
	        [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.range_resolution; })
	    .def_property_readonly(
	        "angular_resolution",
	        [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.angular_resolution; })
	    .def_property_readonly("min_angle",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.min_angle; })
	    .def_property_readonly("max_angle",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.max_angle; })
	    .def_property_readonly("nrays",
	                           [](const mujoco_ros::sensors::laser::LaserConfig &config) { return config.nrays; })
	    .def("__repr__", [](const mujoco_ros::sensors::laser::LaserConfig &config) {
		    return "<LaserConfig name='" + config.name + "'>";
	    });

	py::class_<LaserPluginHandle, std::shared_ptr<LaserPluginHandle>>(module, "LaserPlugin")
	    .def_property_readonly("configs",
	                           [](const LaserPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::sensors::laser::LaserPlugin>(
		                               [](const auto &plugin) { return plugin.GetLaserConfigs(); });
	                           })
	    .def_property_readonly("config_count",
	                           [](const LaserPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::sensors::laser::LaserPlugin>(
		                               [](const auto &plugin) { return plugin.GetLaserConfigs().size(); });
	                           })
	    .def_property_readonly("has_render_data",
	                           [](const LaserPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::sensors::laser::LaserPlugin>(
		                               [](const auto &plugin) { return plugin.HasRenderData(); });
	                           })
	    .def_property_readonly("render_geom_count",
	                           [](const LaserPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::sensors::laser::LaserPlugin>(
		                               [](const auto &plugin) { return plugin.GetRenderGeomCount(); });
	                           })
	    .def("__repr__", [](const LaserPluginHandle &plugin) {
		    return "<LaserPlugin name='" + plugin.handle.Name() + "' type='" + plugin.handle.Type() + "'>";
	    });

	module.def("bind", &Bind, py::arg("plugin"));
}

} // namespace mujoco_ros::python::laser
