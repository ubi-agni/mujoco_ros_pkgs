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
