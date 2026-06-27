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

#include <array>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

namespace py = pybind11;

namespace mujoco_ros::python::sensors {
namespace {

std::array<double, 3> CopyArray(const double values[3])
{
	return { values[0], values[1], values[2] };
}

py::dict ConfigsDict(mujoco_ros::sensors::MujocoRosSensorsPlugin &plugin)
{
	py::dict configs;
	for (const auto &pair : plugin.GetSensorConfigs()) {
		configs[py::str(pair.first)] = py::cast(pair.second.get(), py::return_value_policy::reference);
	}
	return configs;
}

} // namespace

mujoco_ros::sensors::MujocoRosSensorsPlugin *Bind(mujoco_ros::MujocoPlugin *plugin)
{
	auto *typed_plugin = dynamic_cast<mujoco_ros::sensors::MujocoRosSensorsPlugin *>(plugin);
	if (typed_plugin == nullptr) {
		throw py::type_error("plugin is not a MujocoRosSensorsPlugin");
	}
	return typed_plugin;
}

PYBIND11_MODULE(pymujoco_ros_sensors, module)
{
	py::module_::import("pymujoco_ros");

	py::class_<mujoco_ros::sensors::SensorConfig>(module, "_SensorConfig")
	    .def_property_readonly("frame_id",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return config.frame_id; })
	    .def_property_readonly("mean",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return CopyArray(config.mean); })
	    .def_property_readonly("sigma",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return CopyArray(config.sigma); })
	    .def_property_readonly("is_set",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return config.is_set != 0; })
	    .def("__repr__", [](const mujoco_ros::sensors::SensorConfig &config) {
		    std::ostringstream stream;
		    stream << "<SensorConfig frame_id='" << config.frame_id << "' mean=(" << config.mean[0] << ", "
		           << config.mean[1] << ", " << config.mean[2] << ") sigma=(" << config.sigma[0] << ", "
		           << config.sigma[1] << ", " << config.sigma[2] << ") is_set=" << (config.is_set != 0) << ">";
		    return stream.str();
	    });

	py::class_<mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin,
	           std::shared_ptr<mujoco_ros::sensors::MujocoRosSensorsPlugin>>(module, "MujocoRosSensorsPlugin")
	    .def_property_readonly("configs", &ConfigsDict)
	    .def("configs_dict", &ConfigsDict)
	    .def_property_readonly(
	        "config_count",
	        [](const mujoco_ros::sensors::MujocoRosSensorsPlugin &plugin) { return plugin.GetSensorConfigs().size(); })
	    .def("__repr__", [](const mujoco_ros::sensors::MujocoRosSensorsPlugin &plugin) {
		    return "<MujocoRosSensorsPlugin name='" + plugin.get_name() + "' type='" + plugin.get_type() + "'>";
	    });

	module.def("bind", &Bind, py::arg("plugin"), py::return_value_policy::reference);
}

} // namespace mujoco_ros::python::sensors
