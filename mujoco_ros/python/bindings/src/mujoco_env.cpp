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

/* Authors: David P. Leins*/

#include "pymujoco_ros.hpp"

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/render_backend.hpp>

#include <pybind11/stl.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <ros/ros.h>
#else
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <mujoco_ros/offscreen_camera.hpp>
#if RENDER_BACKEND == GLFW_BACKEND
#include <mujoco_ros/glfw_adapter.h>
#include <mujoco_ros/viewer.hpp>
#endif

namespace mujoco_ros::python {
namespace {

std::string NormalizeNamespace(std::string ns)
{
	while (ns.find("//") != std::string::npos) {
		ns.replace(ns.find("//"), 2, "/");
	}
	if (ns.size() > 1 && ns.back() == '/') {
		ns.pop_back();
	}
	return ns;
}

#if MJR_ROS_VERSION == ROS_1
void EnsureRosInitialized()
{
	if (!ros::isInitialized()) {
		ros::M_string remappings;
		ros::init(remappings, "pymujoco_ros", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	}
}
#else
std::vector<std::string> GetPythonArgv()
{
	std::vector<std::string> args;
	try {
		args = py::module_::import("sys").attr("argv").cast<std::vector<std::string>>();
	} catch (const py::error_already_set &) {
		args.clear();
	}
	if (args.empty()) {
		args.emplace_back("pymujoco_ros");
	}
	return args;
}

void EnsureRosInitialized()
{
	if (!rclcpp::ok()) {
		auto args = GetPythonArgv();
		std::vector<char *> argv;
		argv.reserve(args.size());
		for (auto &arg : args) {
			argv.emplace_back(arg.data());
		}
		int argc = static_cast<int>(argv.size());
		rclcpp::init(argc, argv.data());
	}
}
#endif

class MujocoEnvWrapper : public MujocoEnv
{
public:
#if MJR_ROS_VERSION == ROS_1
	explicit MujocoEnvWrapper(const std::string &admin_hash = std::string())
	    : MujocoEnv((EnsureRosInitialized(), admin_hash))
	{
	}
	MujocoEnvWrapper(const std::string &admin_hash, bool python_reload_service)
	    : MujocoEnv((EnsureRosInitialized(), admin_hash), python_reload_service)
	{
	}
#else
	explicit MujocoEnvWrapper(const std::string &admin_hash = std::string(), bool python_reload_service = false)
	    : MujocoEnv(MakeExecutor(), admin_hash, false, python_reload_service)
	{
		GetExecutorPtr()->add_node(get_node_base_interface());
		executor_thread_handle_ = std::thread([this]() { GetExecutorPtr()->spin(); });
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		Configure();
		construction_complete_ = true;
	}
#endif

	~MujocoEnvWrapper() override
	{
		MJR_DEBUG("MujocoEnvWrapper destructor called");
		try {
			ShutdownAndJoin();
		} catch (...) {
		}
	}

	void StartPhysics()
	{
		if (physics_started_) {
			return;
		}
		MujocoEnv::StartPhysicsLoop();
		physics_started_ = true;
	}

	void StartEvents()
	{
		if (events_started_) {
			return;
		}
		MujocoEnv::StartEventLoop();
		events_started_ = true;
	}

	bool LoadModel(const std::string &model_or_path)
	{
		StartPhysics();
		StartEvents();
		return MujocoEnv::LoadModelFromString(model_or_path);
	}

	bool LoadPythonModel(py::object model, py::object data, const std::string &filename, double timeout)
	{
		StartPhysics();
		StartEvents();
		if (!model_py_.is_none() || !data_py_.is_none()) {
			retained_python_models_.emplace_back(model_py_, data_py_);
		}
		model_py_ = std::move(model);
		data_py_  = std::move(data);

		const auto model_address = model_py_.attr("_address").cast<std::uintptr_t>();
		const auto data_address  = data_py_.attr("_address").cast<std::uintptr_t>();

		{
			RecursiveLock lock(physics_thread_mutex_);
			mnew = reinterpret_cast<mjModel *>(model_address);
			dnew = reinterpret_cast<mjData *>(data_address);
			std::strncpy(filename_, filename.c_str(), kMaxFilenameLength - 1);
			filename_[kMaxFilenameLength - 1] = '\0';
			settings_.is_python_request.store(1);
		}
		RequestLoad(1);

		const auto deadline = Clock::now() + Seconds(timeout);
		while (GetControlSnapshot().load_request != 0) {
			if (Clock::now() > deadline) {
				RecursiveLock lock(physics_thread_mutex_);
				RequestLoad(0);
				settings_.is_python_request.store(0);
				mnew = nullptr;
				dnew = nullptr;
				throw std::runtime_error("Timed out while loading Python-owned MuJoCo model/data");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		return sim_state_.model_valid;
	}

	bool AttachViewer(bool active)
	{
		if (!active) {
			throw std::runtime_error("Passive viewer attachment is not implemented");
		}
#if RENDER_BACKEND == GLFW_BACKEND
		if (viewer_running_) {
			return false;
		}
		viewer_thread_handle_ = std::thread([this]() {
			auto adapter     = std::make_unique<mujoco_ros::GlfwAdapter>();
			auto viewer      = std::make_unique<mujoco_ros::Viewer>(std::move(adapter), this, false);
			attached_viewer_ = viewer.get();
			viewer_running_  = true;
			viewer->RenderLoop();
			attached_viewer_ = nullptr;
			viewer_running_  = false;
		});
		return true;
#else
		throw std::runtime_error("Viewer attachment requires the GLFW render backend");
#endif
	}

	bool StepSimulation(int num_steps, bool blocking) { return MujocoEnv::Step(num_steps, blocking); }

	void ResetSimulation() { MujocoEnv::Reset(); }

	bool TogglePausedWrapper(bool paused, const std::string &admin_hash)
	{
		return MujocoEnv::TogglePaused(paused, admin_hash);
	}

	bool SetRealTimeFactorWrapper(float real_time_factor, const std::string &admin_hash)
	{
		return MujocoEnv::SetRealTimeFactor(real_time_factor, admin_hash);
	}

	void SetBusywaitWrapper(int busywait)
	{
		RecursiveLock lock(physics_thread_mutex_);
		settings_.busywait = busywait;
		settings_.settings_changed.store(1);
	}

	std::array<double, 3> GetGravityWrapper()
	{
		mjtNum gravity[3] = { 0, 0, 0 };
		char status[MujocoEnv::kErrorLength];
		if (!MujocoEnv::GetGravity(gravity, "", status, MujocoEnv::kErrorLength)) {
			throw std::runtime_error(status);
		}
		return { static_cast<double>(gravity[0]), static_cast<double>(gravity[1]), static_cast<double>(gravity[2]) };
	}

	bool SetGravityWrapper(const std::vector<double> &gravity, const std::string &admin_hash)
	{
		if (gravity.size() != 3) {
			throw std::invalid_argument("gravity must contain exactly three values");
		}
		mjtNum gravity_values[3] = { static_cast<mjtNum>(gravity[0]), static_cast<mjtNum>(gravity[1]),
			                          static_cast<mjtNum>(gravity[2]) };
		return MujocoEnv::SetGravity(gravity_values, admin_hash);
	}

	py::object ModelPy() const { return model_py_; }

	py::object DataPy() const { return data_py_; }

	void ShutdownAndJoin()
	{
		const bool should_request_shutdown = !shutdown_called_;
		shutdown_called_                   = true;

		if (should_request_shutdown) {
			try {
				MujocoEnv::Shutdown();
			} catch (...) {
			}
		}
		try {
			ShutdownViewer();
		} catch (...) {
		}
		try {
			MujocoEnv::WaitForPhysicsJoin();
		} catch (...) {
		}
		try {
			JoinOffscreenRenderThread();
		} catch (...) {
		}
		try {
			MujocoEnv::WaitForEventsJoin();
		} catch (...) {
		}
		try {
			DetachPythonOwnedModel();
		} catch (...) {
		}

#if MJR_ROS_VERSION == ROS_2
		const auto executor = GetExecutorPtr();
		if (executor != nullptr) {
			try {
				executor->cancel();
			} catch (...) {
			}
		}
		if (executor_thread_handle_.joinable()) {
			executor_thread_handle_.join();
		}
		if (construction_complete_) {
			try {
				if (executor != nullptr) {
					executor->remove_node(get_node_base_interface());
				}
			} catch (...) {
			}
			construction_complete_ = false;
		}
#endif
	}

	bool ModelValid() const { return sim_state_.model_valid; }

	unsigned int LoadCount() const { return sim_state_.load_count; }

	int OperationalStatus() { return MujocoEnv::GetOperationalStatus(); }

	EnvSettings Settings() const { return MujocoEnv::GetSettings(); }

	SimState State() const { return MujocoEnv::GetSimState(); }

	SimInfo Info() { return MujocoEnv::GetSimInfo(); }

	std::vector<PluginStat> PluginStats() { return MujocoEnv::GetPluginStats(); }

	std::vector<std::string> PluginNames() const
	{
		std::vector<std::string> plugins;
		for (const auto &plugin : MujocoEnv::GetPlugins()) {
			plugins.emplace_back(plugin->get_name());
		}
		return plugins;
	}

	OffscreenRenderContext &Offscreen() { return offscreen_; }

	std::vector<MujocoPlugin *> PluginObjects() const
	{
		std::vector<MujocoPlugin *> plugins;
		for (const auto &plugin : MujocoEnv::GetPlugins()) {
			plugins.emplace_back(plugin.get());
		}
		return plugins;
	}

	std::string Filename() const { return std::string(filename_); }

	bool IsRunning() const { return GetControlSnapshot().running; }

	std::string HandleNamespace() const
	{
#if MJR_ROS_VERSION == ROS_1
		return nh_ ? nh_->getNamespace() : std::string();
#else
		return NormalizeNamespace(std::string(get_effective_namespace()) + "/" + std::string(get_name()));
#endif
	}

private:
	static std::uintptr_t PythonMujocoAddress(const py::object &object)
	{
		if (object.is_none()) {
			return 0;
		}
		return object.attr("_address").cast<std::uintptr_t>();
	}

	void DetachPythonOwnedModel()
	{
		const auto current_model_address = reinterpret_cast<std::uintptr_t>(model_.get());
		const auto current_data_address  = reinterpret_cast<std::uintptr_t>(data_.get());
		const bool model_is_python_owned =
		    current_model_address != 0 && current_model_address == PythonMujocoAddress(model_py_);
		const bool data_is_python_owned =
		    current_data_address != 0 && current_data_address == PythonMujocoAddress(data_py_);

		if (model_is_python_owned || data_is_python_owned) {
			RecursiveLock lock(physics_thread_mutex_);
			cb_ready_plugins_.clear();
			plugins_.clear();
			offscreen_.cams.clear();
			model_.reset();
			data_.reset();
		}

		model_py_ = py::none();
		data_py_  = py::none();
		retained_python_models_.clear();
	}

	void ShutdownViewer()
	{
#if RENDER_BACKEND == GLFW_BACKEND
		if (attached_viewer_ != nullptr) {
			attached_viewer_->exit_request.store(1);
		}
		if (viewer_thread_handle_.joinable()) {
			viewer_thread_handle_.join();
		}
		viewer_running_  = false;
		attached_viewer_ = nullptr;
#endif
	}

	void JoinOffscreenRenderThread()
	{
		if (offscreen_.render_thread_handle.joinable()) {
			offscreen_.cond_render_request.notify_one();
			offscreen_.render_thread_handle.join();
		}
	}

#if MJR_ROS_VERSION == ROS_2
	static rclcpp::Executor::SharedPtr MakeExecutor()
	{
		EnsureRosInitialized();
		return std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	}
#endif

	bool physics_started_ = false;
	bool events_started_  = false;
	bool shutdown_called_ = false;
	py::object model_py_  = py::none();
	py::object data_py_   = py::none();
	std::vector<std::pair<py::object, py::object>> retained_python_models_;
#if RENDER_BACKEND == GLFW_BACKEND
	std::thread viewer_thread_handle_;
	std::atomic_bool viewer_running_     = false;
	mujoco_ros::Viewer *attached_viewer_ = nullptr;
#endif
#if MJR_ROS_VERSION == ROS_2
	bool construction_complete_ = false;
	std::thread executor_thread_handle_;
#endif
};

} // namespace

void InitMujocoEnv(py::module_ &module)
{
	py::class_<MujocoEnvWrapper, std::shared_ptr<MujocoEnvWrapper>>(module, "_MujocoEnvWrapper")
	    .def(py::init([](std::optional<std::string> admin_hash, bool python_reload_service) {
		         return std::make_shared<MujocoEnvWrapper>(admin_hash.value_or(""), python_reload_service);
	         }),
	         py::arg("admin_hash") = py::none(), py::arg("python_reload_service") = false)
	    .def("start_physics_loop", &MujocoEnvWrapper::StartPhysics)
	    .def("start_event_loop", &MujocoEnvWrapper::StartEvents)
	    .def("shutdown", &MujocoEnvWrapper::ShutdownAndJoin)
	    .def("wait_for_physics_join", &MujocoEnvWrapper::WaitForPhysicsJoin)
	    .def("wait_for_events_join", &MujocoEnvWrapper::WaitForEventsJoin)
	    .def("load_model_from_string", &MujocoEnvWrapper::LoadModel, py::arg("model_or_path"))
	    .def("_load", &MujocoEnvWrapper::LoadPythonModel, py::arg("model"), py::arg("data"), py::arg("filename") = "",
	         py::arg("timeout") = 5.0)
	    .def("step", &MujocoEnvWrapper::StepSimulation, py::arg("num_steps") = 1, py::arg("blocking") = true)
	    .def("reset", &MujocoEnvWrapper::ResetSimulation)
	    .def("toggle_paused", &MujocoEnvWrapper::TogglePausedWrapper, py::arg("paused"), py::arg("admin_hash") = "")
	    .def("set_rt_factor", &MujocoEnvWrapper::SetRealTimeFactorWrapper, py::arg("rt_factor"),
	         py::arg("admin_hash") = "")
	    .def("set_busywait", &MujocoEnvWrapper::SetBusywaitWrapper, py::arg("busywait"))
	    .def("attach_viewer", &MujocoEnvWrapper::AttachViewer, py::arg("active") = true)
	    .def("get_gravity", &MujocoEnvWrapper::GetGravityWrapper)
	    .def("set_gravity", &MujocoEnvWrapper::SetGravityWrapper, py::arg("gravity"), py::arg("admin_hash") = "")
	    .def_property_readonly("model_valid", &MujocoEnvWrapper::ModelValid)
	    .def_property_readonly("load_count", &MujocoEnvWrapper::LoadCount)
	    .def_property_readonly("operational_status", &MujocoEnvWrapper::OperationalStatus)
	    .def_property_readonly("settings", &MujocoEnvWrapper::Settings)
	    .def_property_readonly("sim_state", &MujocoEnvWrapper::State)
	    .def_property_readonly("sim_info", &MujocoEnvWrapper::Info)
	    .def_property_readonly("plugin_stats", &MujocoEnvWrapper::PluginStats)
	    .def_property_readonly("plugins", &MujocoEnvWrapper::PluginObjects, py::return_value_policy::reference)
	    .def_property_readonly("plugin_names", &MujocoEnvWrapper::PluginNames)
	    .def_property_readonly("_offscreen_context", &MujocoEnvWrapper::Offscreen,
	                           py::return_value_policy::reference_internal)
	    .def_property_readonly("model", &MujocoEnvWrapper::ModelPy)
	    .def_property_readonly("data", &MujocoEnvWrapper::DataPy)
	    .def_property_readonly("filename", &MujocoEnvWrapper::Filename)
	    .def_property_readonly("handle_namespace", &MujocoEnvWrapper::HandleNamespace)
	    .def_property_readonly("is_running", &MujocoEnvWrapper::IsRunning)
	    .def("__repr__", [](const MujocoEnvWrapper &self) {
		    return std::string("<MujocoEnvWrapper filename='") + self.Filename() + "'>";
	    });
}

} // namespace mujoco_ros::python
