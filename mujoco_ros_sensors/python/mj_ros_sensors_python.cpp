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
PYBIND11_MODULE(_mujoco_ros_sensors_python, m)
{
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
