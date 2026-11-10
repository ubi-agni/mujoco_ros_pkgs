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

#include <any>
#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <py_binding_tools/ros_msg_typecasters.h>

#include <memory>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mujoco_ros_msgs/CamProjectionStamped.h>
#include <mujoco_ros_msgs/GeomFromToStamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>
namespace sensor_msgs_ns {
using AccelStamped         = geometry_msgs::AccelStamped;
using CamProjectionStamped = mujoco_ros_msgs::CamProjectionStamped;
using GeomFromToStamped    = mujoco_ros_msgs::GeomFromToStamped;
using Imu                  = sensor_msgs::Imu;
using Odometry             = nav_msgs::Odometry;
using PointStamped         = geometry_msgs::PointStamped;
using PoseStamped          = geometry_msgs::PoseStamped;
using QuaternionStamped    = geometry_msgs::QuaternionStamped;
using ScalarStamped        = mujoco_ros_msgs::ScalarStamped;
using TwistStamped         = geometry_msgs::TwistStamped;
using Vector3Stamped       = geometry_msgs::Vector3Stamped;
using WrenchStamped        = geometry_msgs::WrenchStamped;
} // namespace sensor_msgs_ns
#else
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mujoco_ros_msgs/msg/cam_projection_stamped.hpp>
#include <mujoco_ros_msgs/msg/geom_from_to_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace sensor_msgs_ns {
using AccelStamped         = geometry_msgs::msg::AccelStamped;
using CamProjectionStamped = mujoco_ros_msgs::msg::CamProjectionStamped;
using GeomFromToStamped    = mujoco_ros_msgs::msg::GeomFromToStamped;
using Imu                  = sensor_msgs::msg::Imu;
using Odometry             = nav_msgs::msg::Odometry;
using PointStamped         = geometry_msgs::msg::PointStamped;
using PoseStamped          = geometry_msgs::msg::PoseStamped;
using QuaternionStamped    = geometry_msgs::msg::QuaternionStamped;
using ScalarStamped        = mujoco_ros_msgs::msg::ScalarStamped;
using TwistStamped         = geometry_msgs::msg::TwistStamped;
using Vector3Stamped       = geometry_msgs::msg::Vector3Stamped;
using WrenchStamped        = geometry_msgs::msg::WrenchStamped;
} // namespace sensor_msgs_ns
#endif

namespace py = pybind11;

namespace mujoco_ros::python::sensors {
namespace {

using SensorsPlugin = mujoco_ros::sensors::MujocoRosSensorsPlugin;

std::array<double, 3> CopyArray(const double values[3])
{
	return { values[0], values[1], values[2] };
}

struct SensorsPluginHandle
{
	explicit SensorsPluginHandle(mujoco_ros::PluginHandle handle) : handle(std::move(handle)) {}
	mujoco_ros::PluginHandle handle;
};

template <typename MsgT>
bool TryCast(const std::any &value, py::object &out)
{
	if (value.type() != typeid(MsgT)) {
		return false;
	}
	out = py::cast(std::any_cast<MsgT>(value));
	return true;
}

/// Message types a native stream can produce; the stream hands them over type-erased.
py::object AnyToPython(const std::any &value)
{
	using namespace sensor_msgs_ns;
	py::object result;
	if (TryCast<Vector3Stamped>(value, result) || TryCast<PointStamped>(value, result) ||
	    TryCast<QuaternionStamped>(value, result) || TryCast<ScalarStamped>(value, result) ||
	    TryCast<CamProjectionStamped>(value, result) || TryCast<GeomFromToStamped>(value, result)) {
		return result;
	}
	// Compound sensors (ADR: compound streams share the native hierarchy).
	if (TryCast<WrenchStamped>(value, result) || TryCast<TwistStamped>(value, result) ||
	    TryCast<AccelStamped>(value, result) || TryCast<PoseStamped>(value, result) || TryCast<Imu>(value, result) ||
	    TryCast<Odometry>(value, result)) {
		return result;
	}
	throw std::runtime_error("sensor stream produced a message type without a Python conversion");
}

/**
 * By-value reference to one stream: plugin handle + name, never a stream pointer.
 * Every access re-resolves under the plugin access (physics_thread_mutex_ held), so after
 * a reload/Clear the generation check fails loud instead of touching a freed stream.
 */
struct SensorStreamHandle
{
	mujoco_ros::PluginHandle handle;
	std::string name;

	template <typename Func>
	decltype(auto) With(Func &&func) const
	{
		return handle.WithBackend<SensorsPlugin>([&](auto &plugin) -> decltype(auto) {
			if (plugin.Find(name) == nullptr) {
				throw py::key_error("sensor '" + name + "' no longer exists");
			}
			return func(plugin);
		});
	}
};

struct SensorMap
{
	std::shared_ptr<SensorsPluginHandle> plugin;
};

std::vector<std::string> Names(const SensorMap &map)
{
	return map.plugin->handle.WithBackend<SensorsPlugin>([](auto &plugin) { return plugin.PublicNames(); });
}

std::shared_ptr<SensorsPluginHandle> Bind(const py::object &plugin)
{
	auto handle = py::cast<mujoco_ros::PluginHandle>(plugin);
	if (handle.Type() != "mujoco_ros_sensors/MujocoRosSensorsPlugin") {
		throw py::type_error("plugin is not a MujocoRosSensorsPlugin");
	}
	return std::make_shared<SensorsPluginHandle>(std::move(handle));
}

} // namespace

PYBIND11_MODULE(pymujoco_ros_sensors, module)
{
	py::module_::import("pymujoco_ros");

	py::class_<SensorMap, std::shared_ptr<SensorMap>>(module, "_SensorMap")
	    .def("__getitem__",
	         [](const SensorMap &map, const std::string &name) {
		         return map.plugin->handle.WithBackend<SensorsPlugin>([&](auto &plugin) {
			         if (plugin.Find(name) == nullptr) {
				         throw py::key_error(name);
			         }
			         return std::make_shared<SensorStreamHandle>(SensorStreamHandle{ map.plugin->handle, name });
		         });
	         })
	    .def("__contains__",
	         [](const SensorMap &map, const std::string &name) {
		         return map.plugin->handle.WithBackend<SensorsPlugin>(
		             [&](auto &plugin) { return plugin.Find(name) != nullptr; });
	         })
	    .def("__len__", [](const SensorMap &map) { return Names(map).size(); })
	    .def("__iter__", [](const SensorMap &map) { return py::iter(py::cast(Names(map))); })
	    .def("keys", [](const SensorMap &map) { return Names(map); })
	    .def("__repr__",
	         [](const SensorMap &map) { return "<SensorMap " + std::string(py::repr(py::cast(Names(map)))) + ">"; });

	py::class_<mujoco_ros::sensors::SensorConfig>(module, "_SensorConfig")
	    .def_property_readonly("frame_id",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return config.frame_id; })
	    .def_property_readonly("mean",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return CopyArray(config.mean); })
	    .def_property_readonly("sigma",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return CopyArray(config.sigma); })
	    .def_property_readonly("is_set",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return config.is_set != 0; })
	    .def_property_readonly("update_rate",
	                           [](const mujoco_ros::sensors::SensorConfig &config) { return config.update_rate; })
	    .def("__repr__", [](const mujoco_ros::sensors::SensorConfig &config) {
		    std::ostringstream stream;
		    stream << "<SensorConfig frame_id='" << config.frame_id << "' mean=(" << config.mean[0] << ", "
		           << config.mean[1] << ", " << config.mean[2] << ") sigma=(" << config.sigma[0] << ", "
		           << config.sigma[1] << ", " << config.sigma[2] << ") is_set=" << (config.is_set != 0) << ">";
		    return stream.str();
	    });

	py::class_<SensorStreamHandle, std::shared_ptr<SensorStreamHandle>>(module, "_SensorStream")
	    .def_property_readonly("name", [](const SensorStreamHandle &stream) { return stream.name; })
	    .def_property_readonly("is_public", [](const SensorStreamHandle &) { return true; })
	    .def_property_readonly("config",
	                           [](const SensorStreamHandle &stream) {
		                           return stream.With([&](auto &plugin) {
			                           // Native sensor or compound; With() already proved the stream exists.
			                           return mujoco_ros::sensors::SensorConfig(*plugin.FindConfig(stream.name));
		                           });
	                           })
	    .def_property_readonly(
	        "data",
	        [](const SensorStreamHandle &stream) {
		        return stream.With([&](auto &plugin) { return AnyToPython(plugin.ReadSensorValue(stream.name)); });
	        },
	        "Latest sensor value as a message. Cached per step until the next LastStage/step: "
	        "edits made while paused (e.g. changing mjData) are NOT visible until you step "
	        "(or otherwise invalidate the cache).")
	    .def_property_readonly(
	        "gt_data",
	        [](const SensorStreamHandle &stream) {
		        return stream.With([&](auto &plugin) { return AnyToPython(plugin.ReadSensorGroundTruth(stream.name)); });
	        },
	        "Latest noise-free ground-truth value as a message. Cached per step until the next "
	        "LastStage/step, like `data`: edits made while paused need a step before fresh values.")
	    .def("__repr__", [](const SensorStreamHandle &stream) { return "<SensorStream name='" + stream.name + "'>"; });

	// Dict-like, read-only view; keys are re-read on every call so it never goes stale silently.
	py::class_<SensorsPluginHandle, std::shared_ptr<SensorsPluginHandle>>(module, "MujocoRosSensorsPlugin")
	    .def_property_readonly("sensors",
	                           [](const std::shared_ptr<SensorsPluginHandle> &plugin) {
		                           return std::make_shared<SensorMap>(SensorMap{ plugin });
	                           })
	    .def_property_readonly("config_count",
	                           [](const SensorsPluginHandle &handle) {
		                           return handle.handle.WithBackend<mujoco_ros::sensors::MujocoRosSensorsPlugin>(
		                               [](const auto &plugin) { return plugin.GetSensorConfigs().size(); });
	                           })
	    .def("__repr__", [](const SensorsPluginHandle &plugin) {
		    return "<MujocoRosSensorsPlugin name='" + plugin.handle.Name() + "' type='" + plugin.handle.Type() + "'>";
	    });

	module.def("bind", &Bind, py::arg("plugin"));
}

} // namespace mujoco_ros::python::sensors
