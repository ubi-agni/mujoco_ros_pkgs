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

#include <optional>

#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.h>

namespace py  = pybind11;
namespace mju = ::mujoco::sample_util;

namespace mujoco_ros::sensors {
class MujocoSensorPluginAccessor
{
public:
	static py::dict getSensorConfigs(mujoco_ros::sensors::MujocoRosSensorsPlugin &plugin)
	{
		py::dict d;
		for (auto &pair : plugin.sensor_map_) {
			d[pair.first.c_str()] = pair.second.get();
		}
		return d;
	}
};

} // namespace mujoco_ros::sensors

namespace mujoco_ros::python::sensors {

using namespace pybind11::literals;
PYBIND11_MODULE(pymujoco_ros_sensors, m)
{
	py::module::import("mujoco_ros"); // Import mujoco_ros to ensure MujocoEnv is registered

	// py::class_<mujoco_ros::sensors::SensorConfig, std::shared_ptr<mujoco_ros::sensors::SensorConfig>>(
	py::class_<mujoco_ros::sensors::SensorConfig>(m, "_SensorConfig")
	    // .def_property_readonly("frame_id", &mujoco_ros::sensors::SensorConfig::frame_id)
	    .def_property_readonly(
	        "mean",
	        [](mujoco_ros::sensors::SensorConfig &self) { return py::array_t<double>(self.mean, self.mean + 3); })
	    .def_property_readonly(
	        "sigma",
	        [](mujoco_ros::sensors::SensorConfig &self) { return py::array_t<double>(self.sigma, self.sigma + 3); })
	    // .def_property_readonly("is_set", &mujoco_ros::sensors::SensorConfig::is_set)
	    .def("__repr__", [](const mujoco_ros::sensors::SensorConfig &d) {
		    return std::string("<SensorConfig\n") + "  frame_id: " + d.frame_id + "\n" + "  mean: [" +
		           std::to_string(d.mean[0]) + ", " + std::to_string(d.mean[1]) + ", " + std::to_string(d.mean[2]) +
		           "]\n" + "  sigma: [" + std::to_string(d.sigma[0]) + ", " + std::to_string(d.sigma[1]) + ", " +
		           std::to_string(d.sigma[2]) + "]\n" + "  is_set: " + std::to_string(d.is_set) + "\n" + ">";
	    });

	// py::bind_map<std::map<std::string, std::unique_ptr<mujoco_ros::sensors::SensorConfig>>>(m, "SensorConfigMap");

	py::class_<mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin,
	           std::shared_ptr<mujoco_ros::sensors::MujocoRosSensorsPlugin>>(m, "MujocoRosSensorsPlugin")
	    .def("configs_dict", &mujoco_ros::sensors::MujocoSensorPluginAccessor::getSensorConfigs);
}

} // namespace mujoco_ros::python::sensors
