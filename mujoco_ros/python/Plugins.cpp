#include "mujoco_ros_python.h"
#include <mujoco_ros/plugin_utils.h>

namespace py = pybind11;
namespace mujoco_ros::python {
using namespace mujoco_ros;

namespace {
struct EnginePluginLoader
{
	EnginePluginLoader()
	{
		// This will register the MuJoCo engine plugins
		mujoco_ros::plugin_utils::registerEnginePlugins(true);
	}
};
} // namespace

static EnginePluginLoader _engine_plugin_loader;

void InitPluginsPy(py::module &m)
{
	py::class_<MujocoPlugin, std::shared_ptr<MujocoPlugin>>(m, "_MujocoPlugin")
	    .def_readonly("type", &MujocoPlugin::type_)
	    .def_readonly("load_time", &MujocoPlugin::load_time_)
	    .def_readonly("reset_time", &MujocoPlugin::reset_time_)
	    .def_readonly("ema_steptime_control", &MujocoPlugin::ema_steptime_control_)
	    .def_readonly("ema_steptime_passive", &MujocoPlugin::ema_steptime_passive_)
	    .def_readonly("ema_steptime_render", &MujocoPlugin::ema_steptime_render_)
	    .def_readonly("ema_steptime_last_stage", &MujocoPlugin::ema_steptime_last_stage_)
	    .def("__repr__",
	         [](const MujocoPlugin &d) { return std::string("<MujocoPlugin ") + "  type: " + d.type_ + " >"; });
}

} // namespace mujoco_ros::python
