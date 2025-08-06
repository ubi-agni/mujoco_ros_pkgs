/* Author: David P. Leins */

#include "pymujoco_ros.h"

#include <optional>

// #include <pybind11/pybind11.h>
// #include <pybind11/pytypes.h>
// #include <pybind11/stl.h>
// #include <pybind11/numpy.h>

#include <math.h>
#include <mujoco/mujoco.h>

// #include <py_binding_tools/ros_msg_typecasters.h>

// #include <mujoco_ros/viewer.h>

namespace mujoco_ros::python {
namespace py = pybind11;

// TODO LIST
// - Reload:
//  - [x] add is_python_bound flag.
//    in this case, methods around creating new model and data objects in C++ should be disabled,
//    because we can only cast from Python to C++
//    - [x] Disable reload service call in C++ if is_python_bound is true
//    - [ ] Serve reload service call in Python that performs load

PYBIND11_MODULE(pymujoco_ros, m)
{
	// m.doc() = "Python bindings for Mujoco ROS environment";
	m.attr("__mujoco_version__") = mj_versionString();

	// // Import mujoco_ros Python modules
	// py::module::import("mujoco_ros.python.env_structs");
	// py::module::import("mujoco_ros.python.mujoco_env");
	// py::module::import("mujoco_ros.python.plugins");
	// py::module::import("mujoco_ros.python.rendering");

	// Initialize the MujocoEnv Python bindings
	InitMujocoEnvPy(m);
	InitEnvSettingsPy(m);
	InitPluginsPy(m);
	InitRenderingPy(m);

	// Add Mujoco version information
}
} // namespace mujoco_ros::python
