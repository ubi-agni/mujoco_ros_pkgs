/* Author: David P. Leins */

#include <pybind11/pybind11.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/array_safety.h>

// #include <mujoco_ros/viewer.h>

namespace mujoco_ros::python {
namespace py  = pybind11;
namespace mju = ::mujoco::sample_util;

class MujocoEnvWrapper : public mujoco_ros::MujocoEnv
{
public:
	MujocoEnvWrapper(bool headless = false, bool render_offscreen = false) : mujoco_ros::MujocoEnv() {}

	bool loadModel(std::string &error_msg, const std::string &filename, std::string &admin_hash /*= ""*/)
	{
		mju::strcpy_arr(filename_, filename.c_str());

		settings_.load_request = 3;

		while (settings_.load_request > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(3));
		}

		error_msg = std::string(load_error_);
		return load_error_[0] != '\0';
	}
};

PYBIND11_MODULE(pymujoco_ros, m)
{
	py::class_<MujocoEnvWrapper>(m, "MujocoEnv")
	    .def(py::init([](bool headless, bool render_offscreen) {
		    return std::make_unique<MujocoEnvWrapper>(headless, render_offscreen);
	    }))
	    .def("loadModel", &MujocoEnvWrapper::loadModel);
}
} // namespace mujoco_ros::python
