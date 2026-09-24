/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
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
 *   * Neither the name of Bielefeld University nor Neura Robotics nor the names of their
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

#include <mujoco_ros/description_converter.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/render_backend.hpp>

#include <pybind11/stl.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros/ros_one/plugin_utils.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#else
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

#include <mujoco_ros/offscreen_camera.hpp>

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

std::vector<RuntimeOptionInput> RuntimeOptionInputsFromDict(const py::dict &values)
{
	std::vector<RuntimeOptionInput> input;
	input.reserve(values.size());
	for (const auto &item : values) {
		const std::string field = py::cast<std::string>(item.first);
		const py::handle value  = item.second;
		if (py::isinstance<py::bool_>(value)) {
			input.push_back({ field, py::cast<bool>(value) });
		} else if (py::isinstance<py::int_>(value)) {
			input.push_back({ field, py::cast<std::int64_t>(value) });
		} else if (py::isinstance<py::float_>(value)) {
			input.push_back({ field, py::cast<double>(value) });
		} else if (py::isinstance<py::str>(value)) {
			input.push_back({ field, py::cast<std::string>(value) });
		} else {
			throw py::type_error("Runtime Options values must be bool, int, float, or string");
		}
	}
	return input;
}

class TempMjbFileGuard
{
public:
	explicit TempMjbFileGuard(const std::string &path) : path_(path) {}
	~TempMjbFileGuard() { std::remove(path_.c_str()); }

private:
	const std::string &path_;
};

#if MJR_ROS_VERSION == ROS_1
void EnsureRosInitialized()
{
	if (!ros::isInitialized()) {
		ros::M_string remappings;
		ros::init(remappings, "pymujoco_ros", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	}
}

ros::console::levels::Level ParseRosOneLogLevel(const std::string &level)
{
	if (level == "debug")
		return ros::console::levels::Debug;
	if (level == "info")
		return ros::console::levels::Info;
	if (level == "warn" || level == "warning")
		return ros::console::levels::Warn;
	if (level == "error")
		return ros::console::levels::Error;
	if (level == "fatal")
		return ros::console::levels::Fatal;
	throw std::invalid_argument("Unknown log level '" + level + "' (expected one of debug, info, warn, error, fatal)");
}

// Python-facing counterpart to the ROS 2 side's sys.argv/--log-level handling
// (EnsureRosInitialized() below): roscpp has no argv-based --log-level
// equivalent, so instead of building CLI tokens up front, this is called
// after ros::init() (via ensure_ros_initialized() in ros_context.py, or the
// EnsureRosInitialized() above as a fallback) to reconfigure loggers
// in place.
void SetRosOneLoggerLevel(const std::string &logger_name, const std::string &level)
{
	EnsureRosInitialized();
	const std::string resolved_name = logger_name.empty() ? ROSCONSOLE_DEFAULT_NAME : logger_name;
	if (!ros::console::set_logger_level(resolved_name, ParseRosOneLogLevel(level))) {
		throw std::runtime_error("Failed to set log level for logger '" + resolved_name + "'");
	}
	ros::console::notifyLoggerLevelsChanged();
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
	    : MujocoEnv((EnsureRosInitialized(), admin_hash), false, false)
	{
	}
	MujocoEnvWrapper(const std::string &admin_hash, bool python_reload_service)
	    : MujocoEnv((EnsureRosInitialized(), admin_hash), python_reload_service, false)
	{
	}
#else
	explicit MujocoEnvWrapper(const std::string &admin_hash = std::string(), bool python_reload_service = false)
	    : MujocoEnv(MakeExecutor(), admin_hash, false, python_reload_service, false)
	{
		GetExecutorPtr()->add_node(get_node_base_interface());
		executor_spin_exited_.store(false, std::memory_order_release);
		executor_thread_handle_ = std::thread([this]() {
			GetExecutorPtr()->spin();
			executor_spin_exited_.store(true, std::memory_order_release);
		});
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
#if RENDER_BACKEND == GLFW_BACKEND
		prepared_viewer_adapter_ = std::make_unique<mujoco_ros::GlfwAdapter>(false);
#endif
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

		QueueModelAndDataForLoad(reinterpret_cast<mjModel *>(model_address), reinterpret_cast<mjData *>(data_address),
		                         filename, true);

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
#if MJR_ROS_VERSION == ROS_2
			auto adapter = std::move(prepared_viewer_adapter_);
			if (!adapter) {
				throw std::runtime_error("Prepared Python viewer context is unavailable");
			}
			adapter->ShowWindow();
#else
			auto adapter = std::make_unique<mujoco_ros::GlfwAdapter>();
#endif
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

	std::string RenderBackpressurePolicyWrapper() const
	{
		return rendering::RenderBackpressurePolicyToString(MujocoEnv::GetRenderBackpressurePolicy());
	}

	void SetRenderBackpressurePolicyWrapper(const std::string &policy)
	{
		const auto status = MujocoEnv::SetRenderBackpressurePolicy(policy);
		if (!status.ok()) {
			throw py::value_error(status.message);
		}
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

	RuntimeOptionsSnapshot RuntimeOptionsWrapper()
	{
		const auto result = MujocoEnv::GetRuntimeOptions();
		if (!result.ok()) {
			throw std::runtime_error(result.error->message);
		}
		return *result.effective;
	}

	RuntimeOptionsSnapshot ApplyRuntimeOptionsWrapper(const py::dict &values)
	{
		const auto result = MujocoEnv::ApplyRuntimeOptions(RuntimeOptionInputsFromDict(values));
		if (!result.ok()) {
			const auto &error         = *result.error;
			const std::string message = (error.field.empty() ? std::string() : error.field + ": ") + error.message;
			if (error.field.empty()) {
				throw std::runtime_error(message);
			}
			throw py::value_error(message);
		}
		return *result.effective;
	}

	void SetPendingRuntimeOptionsWrapper(const py::dict &values)
	{
		try {
			MujocoEnv::SetPendingRuntimeOptions(RuntimeOptionInputsFromDict(values));
		} catch (const std::invalid_argument &error) {
			throw py::value_error(error.what());
		}
	}

	int CountFreeJointsOnBody(const std::string &body_name) const
	{
		RecursiveLock lock(physics_thread_mutex_);
		if (!sim_state_.model_valid || model_.get() == nullptr) {
			throw std::runtime_error("no valid MuJoCo model is loaded");
		}
		if (body_name.empty()) {
			throw py::value_error("body_name must not be empty");
		}
		const int body_id = mj_name2id(model_.get(), mjOBJ_BODY, body_name.c_str());
		if (body_id < 0) {
			throw py::value_error("no body named '" + body_name + "'");
		}
		int count = 0;
		for (int joint_id = 0; joint_id < model_->njnt; ++joint_id) {
			if (model_->jnt_bodyid[joint_id] == body_id && model_->jnt_type[joint_id] == mjJNT_FREE) {
				++count;
			}
		}
		return count;
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
			MujocoEnv::WaitForEventsJoin();
		} catch (...) {
		}
		try {
			DetachPythonOwnedModel();
		} catch (...) {
		}

#if MJR_ROS_VERSION == ROS_2
		const auto executor = GetExecutorPtr();
		// cancel() before spin() enters is discarded by MultiThreadedExecutor::spin()'s
		// spinning.exchange(true). Retry cancel until the spin thread exits; a cancel
		// issued after spin has begun is always honored.
		if (executor_thread_handle_.joinable()) {
			constexpr auto kJoinDeadline = std::chrono::seconds(5);
			const auto deadline          = std::chrono::steady_clock::now() + kJoinDeadline;
			while (!executor_spin_exited_.load(std::memory_order_acquire)) {
				if (std::chrono::steady_clock::now() >= deadline) {
					executor_thread_handle_.detach();
					throw std::runtime_error("MujocoEnvWrapper::ShutdownAndJoin: executor spin thread did not exit "
					                         "within 5s after cancel(); abandon thread and fail loud");
				}
				if (executor != nullptr) {
					try {
						executor->cancel();
					} catch (...) {
					}
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
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
	bool WaitForOperationalStatusIdle(int timeout_ms)
	{
		return MujocoEnv::WaitForOperationalStatusIdle(std::chrono::milliseconds(timeout_ms));
	}

	EnvSettings Settings() const { return MujocoEnv::GetSettings(); }

	SimState State() const { return MujocoEnv::GetSimState(); }

	SimInfo Info() { return MujocoEnv::GetSimInfo(); }

	std::vector<PluginStat> PluginStats() { return MujocoEnv::GetPluginStats(); }

	std::vector<std::string> PluginNames() const
	{
		std::vector<std::string> plugins;
		for (const auto &stat : MujocoEnv::GetPluginStats()) {
			plugins.emplace_back(stat.name);
		}
		return plugins;
	}

	CameraPublicationTransport &GetCameraPublicationTransport() { return camera_publication_transport_; }

	std::vector<PluginHandle> PluginObjects() const { return GetPluginHandles(); }

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
			plugin_host_->QuiesceAndDestroy();
			camera_publication_transport_.cams.clear();
			model_.reset();
			data_.reset();
		}

		model_py_ = py::none();
		data_py_  = py::none();
		retained_python_models_.clear();
	}

	void ShutdownViewer() {}

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
#if MJR_ROS_VERSION == ROS_2
	std::unique_ptr<mujoco_ros::GlfwAdapter> prepared_viewer_adapter_;
#endif
#endif
#if MJR_ROS_VERSION == ROS_2
	bool construction_complete_ = false;
	std::atomic<bool> executor_spin_exited_{ true };
	std::thread executor_thread_handle_;
#endif
};

} // namespace

void InitMujocoEnv(py::module_ &module)
{
	// First module-level (non-class-bound) function in pymujoco_ros. Reuses
	// SaveDescriptionToTempMjb -- the temp-.mjb-producing helper
	// mujoco_ros::load_model_from_description (description_converter.hpp)
	// itself is built on -- rather than that C++-pointer-returning overload,
	// because there is no existing C++-pointer -> Python-object conversion
	// anywhere in this codebase (only the reverse, via .attr("_address")).
	// Instead this mirrors mujoco_env.py's own _model_from_string/
	// load_from_path idiom (mujoco.MjModel.from_binary_path + mujoco.MjData)
	// from the C++ side, so the returned model/data are genuinely
	// Python-owned mujoco objects, not a hand-rolled pointer wrapper.
	module.def(
	    "load_model_from_description",
	    [](const std::string &urdf_path, const std::string &srdf_path, bool generate_actuators,
	       const std::string &attach_prefix) {
		    std::string tmp_path = mujoco_ros::SaveDescriptionToTempMjb(urdf_path, srdf_path, nullptr, {},
		                                                                generate_actuators, attach_prefix);
		    TempMjbFileGuard tmp_file_guard(tmp_path);
		    py::object mujoco_module = py::module_::import("mujoco");
		    py::object model         = mujoco_module.attr("MjModel").attr("from_binary_path")(tmp_path);
		    py::object data          = mujoco_module.attr("MjData")(model);
		    return py::make_tuple(model, data);
	    },
	    py::arg("urdf_path"), py::arg("srdf_path"), py::arg("generate_actuators") = false,
	    py::arg("attach_prefix") = std::string(""),
	    "Derive a compiled MuJoCo model+data from URDF/SRDF, with no MujocoEnv required. "
	    "Set generate_actuators to derive native MuJoCo actuators from ros2_control command interfaces. "
	    "Set attach_prefix to namespace composed model names.");

#if MJR_ROS_VERSION == ROS_1
	module.def("set_ros1_logger_level", &SetRosOneLoggerLevel, py::arg("logger_name"), py::arg("level"),
	           "Set a roscpp/rosconsole logger to the given level ('debug', 'info', 'warn', 'error', 'fatal') "
	           "and notify roscpp of the change. Initializes roscpp (anonymously) first if not already done.");
#endif

	py::class_<MujocoEnvWrapper, std::shared_ptr<MujocoEnvWrapper>>(module, "_MujocoEnvWrapper")
	    .def(py::init([](std::optional<std::string> admin_hash, bool python_reload_service, py::object runtime_options) {
		         auto wrapper = std::make_shared<MujocoEnvWrapper>(admin_hash.value_or(""), python_reload_service);
		         if (!runtime_options.is_none()) {
			         wrapper->SetPendingRuntimeOptionsWrapper(runtime_options.cast<py::dict>());
		         }
		         return wrapper;
	         }),
	         py::arg("admin_hash") = py::none(), py::arg("python_reload_service") = false,
	         py::arg("runtime_options") = py::none())
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
	    .def_property("render_backpressure_policy", &MujocoEnvWrapper::RenderBackpressurePolicyWrapper,
	                  &MujocoEnvWrapper::SetRenderBackpressurePolicyWrapper)
	    .def("attach_viewer", &MujocoEnvWrapper::AttachViewer, py::arg("active") = true)
	    .def("get_gravity", &MujocoEnvWrapper::GetGravityWrapper)
	    .def("set_gravity", &MujocoEnvWrapper::SetGravityWrapper, py::arg("gravity"), py::arg("admin_hash") = "")
	    .def_property_readonly("runtime_options", &MujocoEnvWrapper::RuntimeOptionsWrapper)
	    .def("apply_runtime_options", &MujocoEnvWrapper::ApplyRuntimeOptionsWrapper, py::arg("values"))
	    .def("count_free_joints_on_body", &MujocoEnvWrapper::CountFreeJointsOnBody, py::arg("body_name"))
	    .def_property_readonly("model_valid", &MujocoEnvWrapper::ModelValid)
	    .def_property_readonly("load_count", &MujocoEnvWrapper::LoadCount)
	    .def_property_readonly("operational_status", &MujocoEnvWrapper::OperationalStatus)
	    .def("wait_for_operational_status_idle", &MujocoEnvWrapper::WaitForOperationalStatusIdle, py::arg("timeout_ms"))
	    .def_property_readonly("settings", &MujocoEnvWrapper::Settings)
	    .def_property_readonly("sim_state", &MujocoEnvWrapper::State)
	    .def_property_readonly("sim_info", &MujocoEnvWrapper::Info)
	    .def_property_readonly("plugin_stats", &MujocoEnvWrapper::PluginStats)
	    .def_property_readonly("plugins", &MujocoEnvWrapper::PluginObjects)
	    .def_property_readonly("plugin_names", &MujocoEnvWrapper::PluginNames)
	    .def_property_readonly("_camera_publication_transport", &MujocoEnvWrapper::GetCameraPublicationTransport,
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
