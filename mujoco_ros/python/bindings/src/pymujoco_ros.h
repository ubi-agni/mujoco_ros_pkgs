#pragma once

#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace mujoco_ros::python {

void InitMujocoEnvPy(py::module &m);
void InitEnvSettingsPy(py::module &m);
void InitPluginsPy(py::module &m);
void InitRenderingPy(py::module &m);

} // namespace mujoco_ros::python
