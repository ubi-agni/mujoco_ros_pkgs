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

/* Authors: David P. Leins */

#include <mujoco/mujoco.h>

#include <cstdio>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/description_converter.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/offscreen_camera.hpp>
#include <mujoco_ros/viewer.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>

#include <mujoco_ros/ros_one/plugin_utils.hpp>

#include <geometry_msgs/TransformStamped.h>
using TransformStamped = geometry_msgs::TransformStamped;

namespace roscpp = ros;
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <utility>
using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace roscpp = rclcpp;
#endif

#if OFFSCREEN_RENDER_BACKEND == OSMESA_BACKEND
static std::string render_backend = "OSMesa";
#elif OFFSCREEN_RENDER_BACKEND == EGL_BACKEND
static std::string render_backend = "EGL";
#else
static std::string render_backend = "NONE. No offscreen rendering available.";
#endif

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

namespace {
class TempMjbFileGuard
{
public:
	explicit TempMjbFileGuard(const std::string &path) : path_(path) {}
	~TempMjbFileGuard() { std::remove(path_.c_str()); }

private:
	const std::string &path_;
};

void ApplyCompiledOffscreenBackendConstraints(EnvSettings &settings)
{
#if OFFSCREEN_RENDER_BACKEND == NO_BACKEND
	if (settings.render_offscreen) {
		MJR_WARN("Offscreen rendering was requested but this build has no offscreen RenderCore backend; "
		         "disabling render_offscreen");
		settings.render_offscreen = false;
	}
#else
	(void)settings;
#endif
}

void AppendCleanupError(std::string &diagnostics, const char *stage, std::exception_ptr error) noexcept
{
	try {
		diagnostics += "; ";
		diagnostics += stage;
		diagnostics += ": ";
		try {
			std::rethrow_exception(error);
		} catch (const std::exception &exception) {
			diagnostics += exception.what();
		} catch (...) {
			diagnostics += "unknown exception";
		}
	} catch (...) {
		// Diagnostics must never prevent the remaining teardown attempts.
	}
}
} // namespace

using Seconds      = std::chrono::duration<double>;
using Milliseconds = std::chrono::duration<double, std::milli>;

MujocoEnv *MujocoEnv::instance = nullptr;

const char *MujocoEnv::Diverged(int disableflags, const mjData *d)
{
	if (disableflags & mjDSBL_AUTORESET) {
		for (mjtWarning w : { mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS }) {
			if (d->warning[w].number > 0) {
				return mju_warningText(w, d->warning[w].lastinfo);
			}
		}
	}

	return nullptr;
}

void MujocoEnv::RunRenderCbs(mjvScene *scene)
{
	plugin_host_->DispatchRender(model_generation_, this->model_.get(), this->data_.get(), scene);
}

void UpdateModelFlags(const mjOption * /*unused*/) {}

void MujocoEnv::RunLastStageCbs()
{
	plugin_host_->DispatchLastStage(model_generation_, this->model_.get(), this->data_.get());
}

#if MJR_ROS_VERSION == ROS_1

MujocoEnv::MujocoEnv(const std::string &admin_hash /* = std::string()*/, bool python_reload_service /* = false */,
                     bool create_gui_adapter /* = true */)
{
	python_reload_service_ = python_reload_service;
	create_gui_adapter_    = create_gui_adapter;
	if (!admin_hash.empty()) {
		mju::strcpy_arr(settings_.admin_hash, admin_hash.c_str());
	} else {
		// make sure hash is empty and null-terminated
		settings_.admin_hash[0] = '\0';
	}

	nh_      = std::make_shared<ros::NodeHandle>("~");
	ros_api_ = std::make_unique<RosAPI>(nh_, this);
	plugin_utils::InitPluginLoader();
	plugin_factory_ = std::make_unique<plugin_utils::RosPluginAdapterFactory>(nh_.get(), this);
	plugin_host_    = std::make_unique<PluginHost>(*plugin_factory_);
	Configure();
}

std::unique_ptr<MujocoEnv> MujocoEnv::from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                       const MeshPrepOptions &mesh_options, bool generate_actuators,
                                                       const std::string &attach_prefix)
{
	std::string tmp_path =
	    SaveDescriptionToTempMjb(urdf_path, srdf_path, nullptr, mesh_options, generate_actuators, attach_prefix);

	TempMjbFileGuard tmp_file_guard(tmp_path);
	auto env = std::make_unique<MujocoEnv>("");
	env->StartPhysicsLoop();
	env->StartEventLoop();

	char load_error[MujocoEnv::kErrorLength] = { '\0' };
	bool ok                                  = env->LoadModelFromString(tmp_path, load_error, sizeof(load_error));

	if (!ok) {
		throw std::runtime_error(std::string("MujocoEnv::from_description: failed to load compiled description: ") +
		                         load_error);
	}
	return env;
}

#else // MJR_ROS_VERSION == ROS_2

MujocoEnv::MujocoEnv(rclcpp::Executor::SharedPtr executor, const std::string &admin_hash /* = std::string()*/,
                     bool auto_configure /* = true */, bool python_reload_service /* = false */,
                     bool create_gui_adapter /* = true */)
    : rclcpp::Node("mujoco_server", "", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    , executor_(std::move(executor))
{
	python_reload_service_ = python_reload_service;
	create_gui_adapter_    = create_gui_adapter;
	if (!admin_hash.empty()) {
		mju::strcpy_arr(settings_.admin_hash, admin_hash.c_str());
	} else {
		// make sure hash is empty and null-terminated
		settings_.admin_hash[0] = '\0';
	}

	ros_api_ = std::make_unique<RosAPI>(this);
	plugin_utils::InitPluginLoader();
	plugin_factory_ = std::make_unique<plugin_utils::RosPluginAdapterFactory>(this);
	plugin_host_    = std::make_unique<PluginHost>(*plugin_factory_);
	if (auto_configure) {
		Configure();
	}
}

void MujocoEnv::AddNodeToExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
	executor_->add_node(std::move(node));
}

void MujocoEnv::RemoveNodeFromExecutor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node)
{
	executor_->remove_node(std::move(node));
}

rclcpp::Executor::SharedPtr MujocoEnv::GetExecutorPtr()
{
	return executor_;
}

std::unique_ptr<MujocoEnv> MujocoEnv::from_description(const std::string &urdf_path, const std::string &srdf_path,
                                                       const MeshPrepOptions &mesh_options, bool generate_actuators,
                                                       const std::string &attach_prefix)
{
	std::string tmp_path =
	    SaveDescriptionToTempMjb(urdf_path, srdf_path, nullptr, mesh_options, generate_actuators, attach_prefix);

	TempMjbFileGuard tmp_file_guard(tmp_path);
	auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
	auto env      = std::make_unique<MujocoEnv>(executor);
	env->StartPhysicsLoop();
	env->StartEventLoop();

	char load_error[MujocoEnv::kErrorLength] = { '\0' };
	bool ok                                  = env->LoadModelFromString(tmp_path, load_error, sizeof(load_error));

	if (!ok) {
		throw std::runtime_error(std::string("MujocoEnv::from_description: failed to load compiled description: ") +
		                         load_error);
	}
	return env;
}

#endif

void MujocoEnv::Configure()
{
	MJR_DEBUG("Configuring simulation server");

	FetchRosConfiguration();
	ros_api_->SetupServices();

	if (settings_.eval_mode) {
		MJR_INFO("Running in evaluation mode. Parsing admin hash...");
		if (!settings_.admin_hash[0]) {
			MJR_ERROR("Evaluation mode requires a hash to verify critical operations are allowed. No hash was provided, "
			          "aborting launch.");
			RequestShutdown();
			throw std::runtime_error(
			    "Evaluation mode requires a hash to verify critical operations are allowed. No hash was "
			    "provided, aborting launch.");
		}
	}

	MJR_DEBUG_COND(!settings_.use_sim_time, "use_sim_time is set to false. Not publishing sim time to /clock!");

	MJR_INFO_STREAM("MuJoCo ROS " << MJR_PROJECT_VERSION << " (" << MJR_GIT_DESCRIBE << (MJR_GIT_DIRTY ? ", dirty" : "")
	                              << "), MuJoCo " << mj_versionString() << ", render backend " << render_backend);
	if (mjVERSION_HEADER != mj_version()) {
		MJR_WARN_STREAM("Headers and library have different versions (headers: " << mjVERSION_HEADER
		                                                                         << ", library: " << mj_version() << ")");
	}

	if (!settings_.headless && create_gui_adapter_) {
#if RENDER_BACKEND == GLFW_BACKEND
		gui_adapter_ = new mujoco_ros::GlfwAdapter();
#else
		MJR_ERROR("Compiled without GLFW support. Cannot run in non-headless mode.");
#endif
	}
	ros_api_->SetupClockPublisher();

	MJR_INFO_COND(!GetControlSnapshot().running, "Starting Simulation in paused mode");

	mjv_defaultScene(&scn_);
	mjv_defaultPerturb(&pert_);

	ApplyCompiledOffscreenBackendConstraints(settings_);

	if (settings_.render_offscreen) {
		// Backend construction is inert. GLFW context ownership starts later on RenderCore's render thread.
		render_core_             = std::make_shared<rendering::RenderCore>(rendering::CreateRenderBackend());
		const auto policy_status = render_core_->SetRenderBackpressurePolicy(settings_.render_backpressure_policy);
		if (!policy_status.ok()) {
			throw std::runtime_error("failed to configure render backpressure policy: " + policy_status.message);
		}
	}

	MJR_INFO_STREAM_COND(num_steps_until_exit_ > 0, "Sim will terminate after " << num_steps_until_exit_ << " steps");

	int available_threads = std::thread::hardware_concurrency() - 1;
	int num_threads       = settings_.num_mj_threads;
	MJR_WARN_STREAM("The 'num_mj_threads' parameter, exposed by the 'mujoco_threads' launch argument, is deprecated. "
	                "MuJoCo upstream is changing the public threading API to internal-only usage, so this behavior "
	                "will change in the future.");
	MJR_WARN_STREAM_COND(num_threads > 1,
	                     "Using more than one MuJoCo thread can increase CPU usage because MuJoCo worker threads "
	                         << "busy-wait. See https://github.com/google-deepmind/mujoco/pull/2746");
	num_threads = std::min(num_threads, available_threads);
	if (num_threads > 1) {
		threadpool_ = mju_threadPoolCreate(num_threads);
		MJR_INFO_STREAM("Using MuJoCo threadpool size of " << num_threads << " (max available: " << available_threads
		                                                   << ")");
	} else {
		MJR_INFO_STREAM("Running MuJoCo in single-threaded mode (" << available_threads << " threads available)");
	}

	// init VFS
	mj_defaultVFS(&vfs_);

	camera_publication_transport_.BindRenderOwner(this);

	// setupServices();

	MujocoEnv::instance = this;

	mjcb_control = ProxyControlCB;
	mjcb_passive = ProxyPassiveCB;

	InitTFBroadcasting();
}

rendering::FrameStatus MujocoEnv::GetRenderStatus() const
{
	if (!render_core_) {
		return rendering::FrameStatus{ rendering::FrameStatusCode::kStopped, 0, std::nullopt, FrameGeneration(0),
			                            "RenderCore is not configured" };
	}
	const auto core_status = render_core_->Status();
	if (!core_status.ok()) {
		return core_status;
	}
	return camera_publication_transport_.LastRenderStatus();
}

rendering::FrameStatus MujocoEnv::SetRenderBackpressurePolicy(rendering::RenderBackpressurePolicy policy)
{
	if (policy != rendering::RenderBackpressurePolicy::kDrop &&
	    policy != rendering::RenderBackpressurePolicy::kWaitForSlot) {
		return rendering::FrameStatus{ rendering::FrameStatusCode::kInvalidPolicy, 0, std::nullopt, frame_generation_,
			                            "render backpressure policy is invalid" };
	}
	std::lock_guard<std::mutex> lock(render_policy_mutex_);
	if (render_core_) {
		const auto status = render_core_->SetRenderBackpressurePolicy(policy);
		if (!status.ok()) {
			return status;
		}
	}
	settings_.render_backpressure_policy = policy;
	return rendering::FrameStatus::Ok();
}

rendering::FrameStatus MujocoEnv::SetRenderBackpressurePolicy(const std::string &policy)
{
	const auto parsed = rendering::RenderBackpressurePolicyFromString(policy);
	if (!parsed.has_value()) {
		return rendering::FrameStatus{ rendering::FrameStatusCode::kInvalidPolicy, 0, std::nullopt, FrameGeneration(0),
			                            "render backpressure policy must be 'drop' or 'wait_for_slot'" };
	}
	return SetRenderBackpressurePolicy(*parsed);
}

rendering::RenderBackpressurePolicy MujocoEnv::GetRenderBackpressurePolicy() const
{
	std::lock_guard<std::mutex> lock(render_policy_mutex_);
	if (render_core_) {
		return render_core_->GetRenderBackpressurePolicy();
	}
	return settings_.render_backpressure_policy;
}

void MujocoEnv::WarnFrameSlotDrop(const rendering::FrameStatus &status)
{
	if (status.code != rendering::FrameStatusCode::kFrameSlotsExhausted) {
		return;
	}
	std::lock_guard<std::mutex> lock(render_warning_mutex_);
	auto now = std::chrono::steady_clock::now();
#ifdef MJR_BUILD_TESTING
	if (warning_clock_for_testing_) {
		now = warning_clock_for_testing_();
	}
#endif
	if (last_frame_slot_warning_.time_since_epoch().count() != 0 &&
	    now - last_frame_slot_warning_ < std::chrono::seconds(1)) {
		return;
	}
	last_frame_slot_warning_ = now;
#ifdef MJR_BUILD_TESTING
	frame_slot_warning_count_.fetch_add(1, std::memory_order_relaxed);
	last_frame_slot_warning_message_ =
	    "Dropped render capture because frame storage capacity is exhausted: " + status.message +
	    ". Set render_backpressure_policy=wait_for_slot when frame frequency is important.";
#endif
	MJR_WARN_STREAM("Dropped render capture because frame storage capacity is exhausted: "
	                << status.message
	                << ". Set "
	                   "render_backpressure_policy=wait_for_slot "
	                   "when frame frequency is important.");
}

FrameGeneration MujocoEnv::ActiveFrameGeneration() const
{
	std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
	if (camera_publication_transport_.cams.empty()) {
		return FrameGeneration(0);
	}
	return frame_generation_;
}

void MujocoEnv::RegisterCollisionFunction(int geom_type1, int geom_type2, mjfCollision collision_cb)
{
	if (custom_collisions_.find(std::pair(geom_type1, geom_type2)) != custom_collisions_.end() &&
	    custom_collisions_.find(std::pair(geom_type2, geom_type2)) != custom_collisions_.end()) {
		MJR_WARN_STREAM("A user defined collision callback for collisions between geoms of type "
		                << geom_type1 << " and " << geom_type2
		                << " have already been registered. This might lead to unexpected behavior!");
	} else {
		custom_collisions_.insert(std::pair(geom_type1, geom_type2));
		defaultCollisionFunctions.emplace_back(
		    CollisionFunctionDefault(geom_type1, geom_type2, mjCOLLISIONFUNC[geom_type1][geom_type2]));
	}
	mjCOLLISIONFUNC[geom_type1][geom_type2] = collision_cb;
}

void MujocoEnv::RegisterStaticTransform(TransformStamped &transform)
{
	MJR_DEBUG_STREAM("Registering static transform for frame " << transform.child_frame_id);
	for (auto it = static_transforms_.begin(); it != static_transforms_.end();) {
		if (it->child_frame_id == transform.child_frame_id) {
			MJR_WARN_STREAM("Static transform for child '" << transform.child_frame_id
			                                               << "' already registered. Will overwrite old transform!");
			static_transforms_.erase(it);
			break;
		}
		++it;
	}

	static_transforms_.emplace_back(transform);

	static_broadcaster_->sendTransform(static_transforms_);
}

void MujocoEnv::EventLoop()
{
	MJR_DEBUG("Starting event loop");
	is_event_running_ = 1;
	auto now          = Clock::now();
	auto fps_cap      = Seconds(mujoco_ros::Viewer::render_ui_rate_upper_bound_); // Cap at 60 fps
	while (roscpp::ok() && !IsShutdownRequested()) {
		bool complete_model_load = false;
		bool reset_requested     = false;
		{
			RecursiveLock lock(physics_thread_mutex_);
			now                         = Clock::now();
			const auto control_snapshot = control_state_.Snapshot();

			if (settings_.settings_changed.load()) {
				settings_.settings_changed.store(0);
				ros_api_->UpdateDynamicParams();
			}

			if (control_snapshot.load_request == 1) {
				MJR_DEBUG("Load request received");
				complete_model_load = true;
			} else if (control_snapshot.load_request >= 2) { // Loading mnew and dnew requested
				MJR_DEBUG("Initializing queued model and data");
				if (InitModelFromQueue()) {
					MJR_DEBUG("Init for load done. Requesting next load step");
					RequestLoad(1);
				} else {
					MJR_ERROR("Init for load failed. Aborting load request");
					mj_deleteData(dnew);
					mj_deleteModel(mnew);
					mnew = nullptr;
					dnew = nullptr;
					control_state_.CompleteFailedLoad();
					sim_state_.load_count += 1;
				}
			}

			if (control_snapshot.reset_requested && !complete_model_load) {
				reset_requested = true;
			}
		}

		if (reset_requested) {
			ProcessReset();
		}

		if (complete_model_load) {
			MJR_DEBUG("Loading model outside the physics lock while render resources quiesce");
			try {
				LoadWithModelAndData();
				MJR_DEBUG("Done loading");

				RecursiveLock lock(physics_thread_mutex_);
				mnew                   = nullptr;
				dnew                   = nullptr;
				sim_state_.model_valid = true;
				sim_state_.load_count += 1;
				reload_in_progress_.store(false, std::memory_order_release);
				OpenRenderTurnAdmission();
				control_state_.PublishOperationalIdle();
				OnReloadPhase(ReloadPhase::kNewGenerationLoaded);
			} catch (const std::exception &error) {
				try {
					MJR_ERROR_STREAM("Model reload failed; entering no-model state: " << error.what());
				} catch (...) {
				}
				HandleReloadFailure(error.what());
			} catch (...) {
				try {
					MJR_ERROR("Model reload failed with an unknown exception; entering no-model state");
				} catch (...) {
				}
				HandleReloadFailure("unknown model reload failure");
			}
		}

		std::this_thread::sleep_for(fps_cap - now.time_since_epoch());
	}
	MJR_DEBUG("Closing all connected viewers");
	for (const auto viewer : connected_viewers_) {
		viewer->exit_request.store(1);
	}
	MJR_DEBUG("Exiting event loop");
	is_event_running_ = 0;
}

void MujocoEnv::ResetOffscreenCameras()
{
	std::vector<rendering::OffscreenCameraPtr> cameras;
	{
		std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
		cameras = camera_publication_transport_.cams;
	}
	for (const auto &cam_ptr : cameras) {
		cam_ptr->Reset();
	}
}

MujocoEnv::InFlightRenderTurn::~InFlightRenderTurn()
{
	Release();
}

void MujocoEnv::InFlightRenderTurn::Release()
{
	if (owner_ != nullptr) {
		owner_->EndInFlightRenderTurn();
		owner_ = nullptr;
	}
}

std::optional<MujocoEnv::InFlightRenderTurn> MujocoEnv::BeginInFlightRenderTurn()
{
	std::lock_guard<std::mutex> lock(render_turn_mutex_);
	if (!render_turn_admission_open_) {
		return std::nullopt;
	}
	++in_flight_render_turn_count_;
	return InFlightRenderTurn(this);
}

void MujocoEnv::EndInFlightRenderTurn()
{
	{
		std::lock_guard<std::mutex> lock(render_turn_mutex_);
		assert(in_flight_render_turn_count_ > 0);
		--in_flight_render_turn_count_;
	}
	render_turn_idle_.notify_all();
}

void MujocoEnv::CloseRenderTurnAdmission()
{
	std::lock_guard<std::mutex> lock(render_turn_mutex_);
	render_turn_admission_open_ = false;
}

void MujocoEnv::OpenRenderTurnAdmission()
{
	std::lock_guard<std::mutex> lock(render_turn_mutex_);
	render_turn_admission_open_ = true;
}

void MujocoEnv::WaitForInFlightRenderTurns()
{
	std::unique_lock<std::mutex> lock(render_turn_mutex_);
	render_turn_idle_.wait(lock, [this] { return in_flight_render_turn_count_ == 0; });
}

bool MujocoEnv::RetireRenderResources(std::string *cleanup_errors) noexcept
{
	try {
		std::vector<rendering::OffscreenCameraPtr> cameras;
		FrameGeneration active_frame_generation;
		std::function<void()> retirement_hook;
		{
			std::lock_guard<std::mutex> lock(physics_test_hook_mutex_);
			retirement_hook = retirement_test_hook_;
		}
		{
			std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
			camera_publication_transport_.retirement_pending = true;
			camera_publication_transport_.UnregisterPythonConsumersLocked();
			cameras                 = camera_publication_transport_.cams;
			active_frame_generation = frame_generation_;
		}
		if (retirement_hook) {
			retirement_hook();
		}
		std::vector<rendering::OffscreenCameraPtr> failed_cameras;
		for (const auto &camera : cameras) {
			try {
				camera->SetActiveRenderCore(std::shared_ptr<rendering::RenderCore>{});
			} catch (...) {
				failed_cameras.push_back(camera);
				if (cleanup_errors != nullptr) {
					AppendCleanupError(*cleanup_errors, "camera retirement", std::current_exception());
				} else {
					try {
						MJR_ERROR("Camera retirement threw during normal reload teardown");
					} catch (...) {
					}
				}
			}
		}

		const bool success = failed_cameras.empty();
		{
			std::lock_guard<std::mutex> lock(camera_publication_transport_.lifecycle_mutex);
			if (success) {
				camera_publication_transport_.cams.clear();
				camera_publication_transport_.retirement_pending = false;
			} else {
				// Keep failed cameras and their old association visible so a failure
				// cleanup pass can retry deactivation without admitting new work.
				camera_publication_transport_.cams = std::move(failed_cameras);
				frame_generation_                  = active_frame_generation;
			}
		}
		return success;
	} catch (...) {
		if (cleanup_errors != nullptr) {
			AppendCleanupError(*cleanup_errors, "camera retirement", std::current_exception());
		} else {
			try {
				MJR_ERROR("Camera retirement failed before lifecycle state could be updated");
			} catch (...) {
			}
		}
		return false;
	}
}

void MujocoEnv::HandleReloadFailure(const char *message) noexcept
{
	std::string diagnostics;
	try {
		diagnostics = message != nullptr ? message : "unknown model reload failure";
	} catch (...) {
		diagnostics.clear();
	}
	std::shared_ptr<rendering::RenderCore> render_core;
	bool resources_safe = true;

	auto attempt = [&diagnostics, &resources_safe](const char *stage, auto &&operation,
	                                               bool resource_teardown = false) noexcept {
		try {
			operation();
		} catch (...) {
			if (resource_teardown) {
				resources_safe = false;
			}
			AppendCleanupError(diagnostics, stage, std::current_exception());
		}
	};

	attempt("close reload admission", [this, &render_core] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		reload_in_progress_.store(true, std::memory_order_release);
		CloseRenderTurnAdmission();
		render_core = render_core_;
	});

	// This must precede model/data clearing and follows the physics -> PluginHost
	// lock order. It also covers adapters loaded by CompleteEnvSetup().
	attempt(
	    "PluginHost quiescence",
	    [this] {
		    RecursiveLock physics_lock(physics_thread_mutex_);
		    if (plugin_host_) {
			    plugin_host_->QuiesceAndDestroy();
		    }
	    },
	    true);

	attempt("pending model cleanup", [this] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		if (mnew != nullptr || dnew != nullptr) {
			if (settings_.is_python_request.load()) {
				settings_.is_python_request.store(0);
			} else {
				mj_deleteData(dnew);
				mj_deleteModel(mnew);
			}
			mnew = nullptr;
			dnew = nullptr;
		}
	});

	attempt(
	    "RenderCore admission stop",
	    [&render_core] {
		    if (render_core) {
			    render_core->StopAcceptingSnapshots();
		    }
	    },
	    true);
	attempt(
	    "RenderCore cancellation",
	    [&render_core] {
		    if (render_core) {
			    render_core->RequestCancelRenderTurn();
		    }
	    },
	    true);
	attempt(
	    "RenderCore completion wait",
	    [&render_core] {
		    if (render_core) {
			    const auto render_status = render_core->FinishOrCancelRenderTurn();
			    if (!render_status.ok()) {
				    throw std::runtime_error("RenderCore failure cleanup was incomplete: " + render_status.message);
			    }
		    }
	    },
	    true);
	attempt("accepted render turn drain", [this] { WaitForInFlightRenderTurns(); }, true);

	attempt(
	    "camera retirement",
	    [this, &diagnostics] {
		    if (!RetireRenderResources(&diagnostics)) {
			    throw std::runtime_error("one or more cameras could not be retired");
		    }
	    },
	    true);
	attempt("failure status recording", [this, &diagnostics] {
		camera_publication_transport_.RecordRenderStatus(rendering::FrameStatus{
		    rendering::FrameStatusCode::kBackendFailure, 0, std::nullopt, FrameGeneration(0), diagnostics });
	});

	attempt("owned model cleanup", [this] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		snapshot_pool_.Deactivate();
		render_model_copy_.reset();
		std::atomic_store(&model_, std::shared_ptr<mjModel>{});
		std::atomic_store(&data_, std::shared_ptr<mjData>{});
	});
	attempt("no-model status", [this, &diagnostics] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		sim_state_.model_valid = false;
		mju::strcpy_arr(load_error_, diagnostics.c_str());
		for (const auto viewer : connected_viewers_) {
			mju::strcpy_arr(viewer->load_error, load_error_);
		}
	});
	attempt("reload admission state", [this, &resources_safe] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		reload_in_progress_.store(false, std::memory_order_release);
		if (resources_safe) {
			OpenRenderTurnAdmission();
		} else {
			try {
				MJR_ERROR("Reload cleanup could not prove resource quiescence; submission admission remains closed");
			} catch (...) {
			}
		}
	});
	attempt("no-model lifecycle state", [this] {
		RecursiveLock physics_lock(physics_thread_mutex_);
		control_state_.CompleteFailedLoad();
	});
	attempt("reload failure observer", [this] { OnReloadPhase(ReloadPhase::kReloadFailed); });
	try {
		MJR_ERROR_STREAM("Reload cleanup completed with failure state: " << diagnostics);
	} catch (...) {
	}
}

void MujocoEnv::ProcessReset()
{
	reset_in_progress_.store(true, std::memory_order_release);
	MJR_DEBUG("Sleeping to ensure all (old) ROS messages are sent outside the physics lock");
	mjtNum reset_time = 0;
	for (;;) {
		WaitForInFlightRenderTurns();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		RecursiveLock lock(physics_thread_mutex_);
		{
			std::lock_guard<std::mutex> turn_lock(render_turn_mutex_);
			if (in_flight_render_turn_count_ != 0) {
				continue;
			}
		}
		if (!model_ || !data_) {
			mju::strcpy_arr(load_error_, "Reset rejected: no model/data is loaded");
			sim_state_.model_valid = false;
			ClearResetRequest();
			reset_in_progress_.store(false, std::memory_order_release);
			MJR_ERROR("Reset rejected while MujocoEnv is in the explicit no-model state");
			std::function<void()> reset_rejected_hook;
			{
				std::lock_guard<std::mutex> hook_lock(physics_test_hook_mutex_);
				reset_rejected_hook = reset_rejected_test_hook_;
			}
			if (reset_rejected_hook) {
				reset_rejected_hook();
			}
			return;
		}
		reset_time = ResetSim();
		break;
	}
	ros_api_->PublishSimTime(reset_time);
	{
		RecursiveLock lock(physics_thread_mutex_);
		ClearResetRequest();
	}
	ResetOffscreenCameras();
	reset_in_progress_.store(false, std::memory_order_release);
}

mjtNum MujocoEnv::ResetSim()
{
	MJR_DEBUG("Resetting simulation environment");

	this->load_error_[0] = '\0';
	mj_resetData(this->model_.get(), this->data_.get());
	LoadInitialJointStates();
	mj_forward(this->model_.get(), this->data_.get());
	const auto reset_time = this->data_->time;

	plugin_host_->Reset(model_generation_);

	for (const auto viewer : connected_viewers_) {
		viewer->reset_request.store(1);
	}
	return reset_time;
}

void MujocoEnv::LoadInitialJointStates()
{
	std::map<std::string, std::vector<double>> pos_map;
	std::map<std::string, std::vector<double>> vel_map;

	GetInitialJointPositions(pos_map);
	GetInitialJointVelocities(vel_map);

	// Change joint positions and velocities in between physics steps
	RecursiveLock lock(physics_thread_mutex_);

	// Joint positions
	for (auto const &[name, axis_vals] : pos_map) {
		MJR_DEBUG_STREAM("Trying to set jointpos of joint " << name);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			MJR_WARN_STREAM("Joint with name '" << name << "' could not be found. Initial joint position cannot be set!");
			continue;
		}

		int num_axes = 0;
		int jnt_type = model_->jnt_type[id];
		switch (jnt_type) {
			case mjJNT_FREE:
				num_axes = 7; // x y z (Position) w x y z (Orientation Quaternion) in world
				break;
			case mjJNT_BALL:
				num_axes = 4; // w x y z (Quaternion)
				break;
			case mjJNT_SLIDE:
			case mjJNT_HINGE:
				num_axes = 1; // single axis value
				break;
			default:
				continue;
		}

		if (axis_vals.size() != num_axes) {
			MJR_ERROR_STREAM("Provided initial position values for joint "
			                 << name << " don't match the degrees of freedom of the joint (exactly " << num_axes
			                 << " values are needed)!");
			continue;
		}
		for (int jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			SetJointPosition(axis_vals[jnt_axis], id, jnt_axis);
		}
	}

	// Joint velocities
	for (auto const &[name, axis_vals] : vel_map) {
		MJR_DEBUG_STREAM("Trying to set jointvels of joint " << name);
		int id = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
		if (id == -1) {
			MJR_WARN_STREAM("Joint with name '" << name << "' could not be found. Initial joint velocity cannot be set!");
			continue;
		}

		int num_axes = 0;
		int jnt_type = model_->jnt_type[id];
		switch (jnt_type) {
			case mjJNT_FREE:
				num_axes = 6; // x y z r p y
				break;
			case mjJNT_BALL:
				num_axes = 3; // r p y
				break;
			case mjJNT_SLIDE:
			case mjJNT_HINGE:
				num_axes = 1; // single axis value
				break;
			default:
				continue;
		}

		if (axis_vals.size() != num_axes) {
			MJR_ERROR_STREAM("Provided initial velocity values for joint "
			                 << name << " don't match the degrees of freedom of the joint (exactly " << num_axes
			                 << " values are needed)!");
			continue;
		}
		for (int jnt_axis = 0; jnt_axis < num_axes; jnt_axis++) {
			SetJointVelocity(axis_vals[jnt_axis], id, jnt_axis);
		}
	}
	// Apply changes in forward dynamics
	mj_forward(model_.get(), data_.get());
}

void MujocoEnv::ConnectViewer(Viewer *viewer)
{
	if (connected_viewers_.empty()) {
		MJR_INFO("Connected first viewer, disabling headless mode");
		settings_.headless = false;
	}
	while (GetOperationalStatus() != 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	MJR_DEBUG("Adding viewer to connected viewers and issuing viewer load request");
	if (std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer) == connected_viewers_.end()) {
		connected_viewers_.emplace_back(viewer);
		if (sim_state_.model_valid) {
			viewer->mnew_ = model_;
			viewer->dnew_ = data_;
			mju::strcmp_arr(viewer->filename, filename_);
			viewer->loadrequest = true;
		}
		return;
	}
	MJR_WARN("Viewer already connected!");
}

void MujocoEnv::DisconnectViewer(Viewer *viewer)
{
	auto it = std::find(connected_viewers_.begin(), connected_viewers_.end(), viewer);
	if (it != connected_viewers_.end()) {
		MJR_DEBUG("Removing viewer from connected viewers");
		connected_viewers_.erase(it);
	} else {
		MJR_WARN("Viewer not connected!");
	}

	if (connected_viewers_.empty()) {
		MJR_INFO_COND(!GetControlSnapshot().shutdown_requested, "Disconnected last viewer, enabling headless mode");
		settings_.headless = true;
	}
}

void MujocoEnv::NotifyGeomChanged(const int geom_id)
{
	plugin_host_->NotifyGeometryChanged(model_generation_, this->model_.get(), this->data_.get(), geom_id);
}

bool MujocoEnv::VerifyAdminHash(const std::string &hash)
{
	if (settings_.eval_mode) {
		MJR_DEBUG("Evaluation mode is active. Checking hash validity");
		if (settings_.admin_hash != hash) {
			return false;
		}
		MJR_DEBUG("Hash valid, request authorized.");
	}
	return true;
}

void MujocoEnv::RunControlCbs(const mjModel *model, mjData *data)
{
	plugin_host_->DispatchControl(model_generation_, model, data);
}

void MujocoEnv::RunPassiveCbs(const mjModel *model, mjData *data)
{
	plugin_host_->DispatchPassive(model_generation_, model, data);
}

MujocoEnv::~MujocoEnv()
{
	MJR_DEBUG("Destructor called");
	is_rendering_running_.store(0);
	plugin_lifetime_.reset();
	// mjcb_control/mjcb_passive are process-wide MuJoCo globals that read MujocoEnv::instance
	// (see ProxyControlCB/ProxyPassiveCB). Left set, a later mj_step/mj_compile call (e.g. from
	// a subsequently-constructed env, or mj_compile's internal warm-up step) would dereference
	// this now-destroyed instance.
	if (MujocoEnv::instance == this) {
		MujocoEnv::instance = nullptr;
		mjcb_control        = nullptr;
		mjcb_passive        = nullptr;
	}
	RequestShutdown();
	if (physics_thread_handle_.joinable()) {
		if (physics_thread_handle_.get_id() == std::this_thread::get_id()) {
			physics_thread_handle_.detach();
		} else {
			MJR_DEBUG("Joining physics thread from destructor");
			physics_thread_handle_.join();
		}
	}
	if (event_thread_handle_.joinable()) {
		if (event_thread_handle_.get_id() == std::this_thread::get_id()) {
			event_thread_handle_.detach();
		} else {
			MJR_DEBUG("Joining event thread from destructor");
			event_thread_handle_.join();
		}
	}

	model_.reset();
	data_.reset();
	connected_viewers_.clear();
	free(this->ctrlnoise_);
	mj_deleteVFS(&vfs_);

	if (threadpool_ != nullptr) {
		mju_threadPoolDestroy(threadpool_);
	}
}

} // namespace mujoco_ros
