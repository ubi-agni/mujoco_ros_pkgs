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

#include <gtest/gtest.h>

#include <atomic>
#include <iomanip>
#include <sstream>

#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <mujoco_ros/SimParamsConfig.h>
#include <mujoco_ros_msgs/EqualityConstraintType.h>

using namespace mujoco_ros_msgs;
#else // MJR_ROS_VERSION == ROS_2
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mujoco_ros_msgs/msg/equality_constraint_type.hpp>
using namespace mujoco_ros_msgs::msg;

namespace ros {
namespace package {
inline std::string getPath(const std::string &package_name)
{
	return ament_index_cpp::get_package_share_directory(package_name);
}
} // namespace package
} // namespace ros

#endif

#include <mujoco_ros/mujoco_env.hpp>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

namespace testing {

#if MJR_ROS_VERSION == ROS_1
using TestNodeHandle = ros::NodeHandle;

#else // MJR_ROS_VERSION == ROS_2
class TestNodeHandle
{
public:
	explicit TestNodeHandle(const std::string &ns = "/mujoco_server") : configured_namespace_(normalize_namespace(ns)) {}

	template <typename T>
	void setParam(const std::string &name, const T &value)
	{
		const std::string ros2_name = normalize_param_name(name);
		rclcpp::Parameter param(ros2_name, value);

		std::lock_guard<std::mutex> lock(param_mutex_);
		desired_params_[ros2_name] = param;

		if (node_ != nullptr) {
			apply_parameters({ param });
		}
	}

	template <typename T>
	bool getParam(const std::string &name, T &value) const
	{
		const std::string ros2_name = normalize_param_name(name);

		std::lock_guard<std::mutex> lock(param_mutex_);

		if (node_ != nullptr) {
			if (!node_->has_parameter(ros2_name)) {
				return false;
			}

			try {
				value = node_->get_parameter(ros2_name).get_value<T>();
				return true;
			} catch (...) {
				return false;
			}
		}

		auto it = desired_params_.find(ros2_name);
		if (it == desired_params_.end()) {
			return false;
		}

		try {
			value = it->second.get_value<T>();
			return true;
		} catch (...) {
			return false;
		}
	}

	void deleteParam(const std::string &name)
	{
		const std::string ros2_name = normalize_param_name(name);

		std::lock_guard<std::mutex> lock(param_mutex_);

		erase_from_desired(ros2_name);

		if (node_ != nullptr) {
			delete_param_from_node(ros2_name);
		}
	}

	void setNode(rclcpp::Node *node)
	{
		if (node == nullptr) {
			throw std::runtime_error("Node pointer cannot be null");
		}

		std::lock_guard<std::mutex> lock(param_mutex_);
		node_ = node;

		std::vector<rclcpp::Parameter> params_to_apply;
		for (const auto &entry : desired_params_) {
			params_to_apply.push_back(entry.second);
		}

		apply_parameters(params_to_apply);
	}

	void clearNode()
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		node_ = nullptr;
	}

	std::string getNamespace() const
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		if (node_ != nullptr) {
			return node_->get_namespace();
		}
		return configured_namespace_;
	}

	std::string getFullyQualifiedNodeName() const
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		if (node_ != nullptr) {
			return node_->get_fully_qualified_name();
		}
		return configured_namespace_;
	}

private:
	void apply_parameters(const std::vector<rclcpp::Parameter> &params)
	{
		if (node_ == nullptr) {
			throw std::runtime_error("Node pointer is null. Cannot apply parameters.");
		}

		for (const auto &param : params) {
			if (!node_->has_parameter(param.get_name())) {
				rcl_interfaces::msg::ParameterDescriptor desc;
				desc.dynamic_typing = true;
				node_->declare_parameter(param.get_name(), param.get_parameter_value(), desc);
			}
		}

		auto result = node_->set_parameters(params);
		for (const auto &param_result : result) {
			if (!param_result.successful) {
				throw std::runtime_error("Failed to set parameters on node: " + param_result.reason);
			}
		}
		// if (!result.success) {
		// 	throw std::runtime_error("Failed to set parameters on node: " + result.reason);
		// }
	}

	void erase_from_desired(const std::string &ros2_name)
	{
		desired_params_.erase(ros2_name);

		auto it = desired_params_.find(ros2_name);
		if (it != desired_params_.end()) {
			desired_params_.erase(it);
		}
	}

	void delete_param_from_node(const std::string &name)
	{
		std::vector<std::string> params_to_delete;

		if (node_->has_parameter(name)) {
			params_to_delete.push_back(name);
		}

		const auto listed = node_->list_parameters({ name }, 0);
		for (const auto &param_name : listed.names) {
			if (param_name == name || param_name.rfind(name + ".", 0) == 0) {
				params_to_delete.push_back(param_name);
			}
		}

		std::sort(params_to_delete.begin(), params_to_delete.end());
		// Remove duplicates just in case
		params_to_delete.erase(std::unique(params_to_delete.begin(), params_to_delete.end()), params_to_delete.end());

		for (const auto &param_name : params_to_delete) {
			if (node_->has_parameter(param_name)) {
				node_->undeclare_parameter(param_name);
			}
		}
	}

	static std::string normalize_namespace(const std::string &ns)
	{
		if (ns.empty() || ns == "~") {
			return "/mujoco_server";
		}
		if (ns.front() != '/') {
			return "/" + ns;
		}
		return ns;
	}

	static std::string normalize_param_name(const std::string &name)
	{
		std::string name_copy = name;
		std::replace(name_copy.begin(), name_copy.end(), '/',
		             '.'); // ROS 2 parameters use '.' instead of '/' as nesting does not exist in the same way as ROS 1
		return name_copy;
	}

	std::string configured_namespace_;
	mutable std::mutex param_mutex_;
	std::map<std::string, rclcpp::Parameter> desired_params_;
	rclcpp::Node *node_{ nullptr };
};

#endif

inline std::string get_test_model_path(const std::string &model_name)
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros_testing_utils") + "/assets/" + model_name;
#else // MJR_ROS_VERSION == ROS_2
	return ament_index_cpp::get_package_share_directory("mujoco_ros_testing_utils") + "/assets/" + model_name;
#endif
}

// Helper struct for topic info abstraction
struct TopicInfo
{
	std::string name;
};

inline bool has_topic(const std::vector<TopicInfo> &topics, const std::string &topic_name)
{
	for (const auto &topic : topics) {
		if (topic.name == topic_name) {
			return true;
		}
	}
	return false;
}

inline bool has_all_topics(const std::vector<TopicInfo> &topics, const std::initializer_list<std::string> &topic_names)
{
	for (const auto &topic_name : topic_names) {
		if (!has_topic(topics, topic_name)) {
			return false;
		}
	}
	return true;
}

// Helper function to get available topics
inline std::vector<TopicInfo> get_available_topics()
{
#if MJR_ROS_VERSION == ROS_1
	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);
	std::vector<TopicInfo> topics;
	for (const auto &t : master_topics) {
		topics.push_back({ t.name });
	}
	return topics;
#else // MJR_ROS_VERSION == ROS_2
	// In ROS 2, topic discovery comes from the node graph.
	std::vector<TopicInfo> topics;
	return topics;
#endif
}

#if MJR_ROS_VERSION == ROS_2
inline std::vector<TopicInfo> get_available_topics(const rclcpp::Node &node)
{
	std::vector<TopicInfo> topics;
	for (const auto &entry : node.get_topic_names_and_types()) {
		topics.push_back({ entry.first });
	}
	return topics;
}
#endif

template <typename NodeT>
inline std::vector<TopicInfo> get_available_topics_for_test(NodeT *node_ptr)
{
#if MJR_ROS_VERSION == ROS_1
	(void)node_ptr;
	return get_available_topics();
#else // MJR_ROS_VERSION == ROS_2
	if (node_ptr == nullptr) {
		return {};
	}
	return get_available_topics(*node_ptr);
#endif
}

template <typename NodeT>
inline std::vector<TopicInfo> get_available_topics_for_test(const std::unique_ptr<NodeT> &node_ptr)
{
	return get_available_topics_for_test(node_ptr.get());
}

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
inline bool wait_for_service(const std::string &service_name)
{
#if MJR_ROS_VERSION == ROS_1
	return ros::service::exists(service_name, true);
#else // MJR_ROS_VERSION == ROS_2
	auto node           = std::make_shared<rclcpp::Node>("mujoco_test_wait_for_service_probe");
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
	while (std::chrono::steady_clock::now() < deadline) {
		for (const auto &entry : node->get_service_names_and_types()) {
			if (entry.first == service_name) {
				return true;
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
#endif
}
#pragma diagnostic pop

#if MJR_ROS_VERSION == ROS_1
template <typename ServiceT>
using ServiceCall = ServiceT;
#else // MJR_ROS_VERSION == ROS_2
template <typename ServiceT>
struct ServiceCall
{
	typename ServiceT::Request request;
	typename ServiceT::Response response;
};
#endif

template <typename NodePtrT>
inline bool service_exists(NodePtrT node_ptr, const std::string &service_name, bool wait_for_discovery = true)
{
#if MJR_ROS_VERSION == ROS_1
	(void)node_ptr;
	return ros::service::exists(service_name, wait_for_discovery);
#else // MJR_ROS_VERSION == ROS_2
	if (node_ptr == nullptr) {
		return false;
	}
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_for_discovery ? 1000 : 10);
	while (std::chrono::steady_clock::now() < deadline) {
		for (const auto &entry : node_ptr->get_service_names_and_types()) {
			if (entry.first == service_name) {
				return true;
			}
		}
		if (!wait_for_discovery) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return false;
#endif
}

template <typename NodePtrT, typename ServiceT>
inline bool call_service(NodePtrT node_ptr, const std::string &service_name, ServiceCall<ServiceT> &service_call)
{
#if MJR_ROS_VERSION == ROS_1
	(void)node_ptr;
	return ros::service::call(service_name, service_call);
#else // MJR_ROS_VERSION == ROS_2
	if (node_ptr == nullptr) {
		return false;
	}
	auto client = node_ptr->template create_client<ServiceT>(service_name);
	if (!client->wait_for_service(std::chrono::seconds(1))) {
		return false;
	}
	auto request = std::make_shared<typename ServiceT::Request>(service_call.request);
	auto future  = client->async_send_request(request);
	if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
		return false;
	}
	service_call.response = *future.get();
	return true;
#endif
}

// Templated test-friendly wrappers that mirror the ROS1 `ros::service::exists/call` signatures
// while delegating to ROS2 node-based implementations when compiled for ROS2.
template <typename NodePtrT>
inline bool service_exists_for_test(NodePtrT node_ptr, const std::string &service_name)
{
#if MJR_ROS_VERSION == ROS_1
	(void)node_ptr;
	// fyi: bool is print_failure_reason
	return ros::service::exists(service_name, true);
#else
	const auto services = node_ptr->get_service_names_and_types();
	return services.find(service_name) != services.end();
#endif
}

template <typename NodePtrT, typename ServiceT>
inline bool service_call_for_test(NodePtrT node_ptr, const std::string &service_name,
                                  ServiceCall<ServiceT> &service_call)
{
#if MJR_ROS_VERSION == ROS_1
	(void)node_ptr;
	return ros::service::call(service_name, service_call);
#else
	return call_service(node_ptr, service_name, service_call);
#endif
}

inline std::vector<double> parse_joint_state_values(const std::string &values)
{
	std::vector<double> parsed_values;
	std::stringstream stream(values);
	double value = 0.0;
	while (stream >> value) {
		parsed_values.push_back(value);
	}
	return parsed_values;
}

inline std::string format_joint_state_values(const std::vector<double> &values)
{
	std::ostringstream stream;
	stream << std::setprecision(17);
	for (std::size_t index = 0; index < values.size(); ++index) {
		if (index > 0) {
			stream << " ";
		}
		stream << values[index];
	}
	return stream.str();
}

inline void set_initial_joint_state_maps(TestNodeHandle *nh, const std::map<std::string, std::string> &pos_map,
                                         const std::map<std::string, std::string> &vel_map)
{
	if (nh == nullptr) {
		return;
	}
#if MJR_ROS_VERSION == ROS_1
	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);
#else // MJR_ROS_VERSION == ROS_2
	nh->deleteParam("initial_joint_states");
	nh->deleteParam("initial_joint_velocities");

	for (const auto &[joint_name, values] : pos_map) {
		nh->setParam("initial_joint_states/" + joint_name, parse_joint_state_values(values));
	}

	for (const auto &[joint_name, values] : vel_map) {
		nh->setParam("initial_joint_velocities/" + joint_name, parse_joint_state_values(values));
	}
#endif
}

inline void clear_initial_joint_state_maps(TestNodeHandle *nh, bool clear_positions = true,
                                           bool clear_velocities = true)
{
	if (nh == nullptr) {
		return;
	}
#if MJR_ROS_VERSION == ROS_1
	if (clear_positions) {
		nh->deleteParam("initial_joint_positions/joint_map");
	}
	if (clear_velocities) {
		nh->deleteParam("initial_joint_velocities/joint_map");
	}
#else // MJR_ROS_VERSION == ROS_2
	if (clear_positions) {
		nh->deleteParam("initial_joint_states");
	}
	if (clear_velocities) {
		nh->deleteParam("initial_joint_velocities");
	}
#endif
}

template <typename NodePtrT>
inline void get_initial_joint_state_maps(NodePtrT node_ptr, TestNodeHandle *nh,
                                         std::map<std::string, std::string> &pos_map,
                                         std::map<std::string, std::string> &vel_map)
{
	pos_map.clear();
	vel_map.clear();

#if MJR_ROS_VERSION == ROS_1
	if (nh == nullptr) {
		return;
	}
	nh->getParam("initial_joint_positions/joint_map", pos_map);
	nh->getParam("initial_joint_velocities/joint_map", vel_map);
#else // MJR_ROS_VERSION == ROS_2
	if (node_ptr == nullptr) {
		return;
	}

	auto pos_list = node_ptr->list_parameters({ "initial_joint_states" }, 2);
	if (!pos_list.names.empty()) {
		auto pos_params = node_ptr->get_parameters(pos_list.names);
		for (const auto &param : pos_params) {
			if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
				const std::string prefix = "initial_joint_states.";
				if (param.get_name().rfind(prefix, 0) == 0) {
					pos_map[param.get_name().substr(prefix.length())] = format_joint_state_values(param.as_double_array());
				}
			}
		}
	}

	auto vel_list = node_ptr->list_parameters({ "initial_joint_velocities" }, 2);
	if (!vel_list.names.empty()) {
		auto vel_params = node_ptr->get_parameters(vel_list.names);
		for (const auto &param : vel_params) {
			if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
				const std::string prefix = "initial_joint_velocities.";
				if (param.get_name().rfind(prefix, 0) == 0) {
					vel_map[param.get_name().substr(prefix.length())] = format_joint_state_values(param.as_double_array());
				}
			}
		}
	}
#endif
}

template <typename NodeT>
inline void get_initial_joint_state_maps(const std::unique_ptr<NodeT> &node_ptr, TestNodeHandle *nh,
                                         std::map<std::string, std::string> &pos_map,
                                         std::map<std::string, std::string> &vel_map)
{
	get_initial_joint_state_maps(node_ptr.get(), nh, pos_map, vel_map);
}

} // namespace testing

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
#if MJR_ROS_VERSION == ROS_1
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string(), ros::NodeHandle *test_nh = nullptr)
	    : MujocoEnv(admin_hash), construction_complete_(true)
	{
	}
	MujocoEnvTestWrapper(ros::NodeHandle * /*test_nh*/) : MujocoEnvTestWrapper("") {}
#else // MJR_ROS_VERSION == ROS_2
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string(), testing::TestNodeHandle *test_nh = nullptr)
	    : MujocoEnv(std::make_shared<rclcpp::executors::MultiThreadedExecutor>(), admin_hash, false)
	    , test_nh_(test_nh)
	    , construction_complete_(false)
	{
		GetExecutorPtr()->add_node(this->get_node_base_interface());

		// Start executor thread BEFORE Configure() so services and callbacks have a spinning executor
		executor_thread_handle_ = std::thread([this]() { GetExecutorPtr()->spin(); });

		// Give executor thread a moment to start spinning
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		try {
			test_nh_->setNode(this);
			Configure();
			construction_complete_ = true;
		} catch (...) {
			MJR_ERROR("Exception thrown during MujocoEnvTestWrapper construction! Cleaning up executor...");
			// Stop executor and join thread BEFORE removing node to avoid races
			GetExecutorPtr()->cancel();
			if (executor_thread_handle_.joinable()) {
				executor_thread_handle_.join();
			}
			GetExecutorPtr()->remove_node(this->get_node_base_interface());
			throw;
		}
	}
	MujocoEnvTestWrapper(testing::TestNodeHandle *test_nh) : MujocoEnvTestWrapper("", test_nh) {}

	~MujocoEnvTestWrapper() override
	{
		try {
			// Only perform cleanup if construction completed successfully
			// If construction failed, base class destructor will handle cleanup
			if (construction_complete_) {
				test_nh_->clearNode();
				shutdown();
			}
		} catch (const std::exception &e) {
			MJR_ERROR_STREAM("Exception during shutdown in destructor: " << e.what());
		} catch (...) {
			MJR_ERROR("Unknown exception during shutdown in destructor");
		}
	}

	testing::TestNodeHandle *test_nh_{ nullptr };
#endif
	std::atomic_bool shutdown_called_{ false };
	std::atomic_bool construction_complete_{ false };
	mjModel *getModelPtr() { return model_.get(); }
	mjData *getDataPtr() { return data_.get(); }
	MujocoEnvMutex *getMutexPtr() { return &physics_thread_mutex_; }

#if MJR_ROS_VERSION == ROS_1
	dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig> *GetParamServer() { return ros_api_->GetParamServerPtr(); }
#else // MJR_ROS_VERSION == ROS_2
	void *GetParamServer() { return nullptr; }
#endif

	int getPendingSteps() { return num_steps_until_exit_; }

	void setEvalMode(bool eval_mode) { settings_.eval_mode = eval_mode; }
	void setAdminHash(const std::string &hash) { mju::strcpy_arr(settings_.admin_hash, hash.c_str()); }

	std::string getFilename() { return { filename_ }; }
	int isPhysicsRunning() { return is_physics_running_; }
	int isEventRunning() { return is_event_running_; }
	int isRenderingRunning() { return is_rendering_running_; }

	OffscreenRenderContext *getOffscreenContext() { return &offscreen_; }

	std::vector<MujocoPluginPtr> const &GetPlugins() const { return MujocoEnv::GetPlugins(); }

	int GetNumCBReadyPlugins() { return cb_ready_plugins_.size(); }
	void NotifyGeomChange() { NotifyGeomChanged(0); }

	bool step(int num_steps = 1, bool blocking = true) { return MujocoEnv::Step(num_steps, blocking); }
	bool togglePaused(bool paused, const std::string &admin_hash = std::string())
	{
		return TogglePaused(paused, admin_hash);
	}
	bool isRunning() const { return GetControlSnapshot().running; }
	int pendingManualSteps() const { return GetControlSnapshot().pending_steps; }
	bool isResetRequested() const { return GetControlSnapshot().reset_requested; }
	bool isShutdownRequested() const { return GetControlSnapshot().shutdown_requested; }
	int loadRequest() const { return GetControlSnapshot().load_request; }
	void requestReset() { RequestReset(); }
	void requestShutdown() { RequestShutdown(); }
	void requestLoad(int load_request) { RequestLoad(load_request); }
	int GetOperationalStatus() { return MujocoEnv::GetOperationalStatus(); }
	void StartPhysicsLoop() { MujocoEnv::StartPhysicsLoop(); }
	void StartEventLoop() { MujocoEnv::StartEventLoop(); }
	void WaitForPhysicsJoin() { MujocoEnv::WaitForPhysicsJoin(); }
	void WaitForEventsJoin() { MujocoEnv::WaitForEventsJoin(); }

	void load_queued_model()
	{
		requestLoad(2);
		float seconds = 0;
		while (GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_LT(seconds, 2) << "Model could not be loaded in time, ran into 2 second timeout!";
	}

	void load_filename(const std::string &filename)
	{
		mju::strcpy_arr(queued_filename_, filename.c_str());
		load_queued_model();
	}

	void shutdown()
	{
		// Guard against multiple shutdown calls (prevent double-shutdown races)
		bool expected = false;
		if (!shutdown_called_.compare_exchange_strong(expected, true)) {
			return; // Already shutting down or already shut down
		}

		// Only clean up physics threads if construction completed successfully
		if (construction_complete_) {
			requestShutdown();
			MujocoEnv::WaitForPhysicsJoin();
			MujocoEnv::WaitForEventsJoin();
		}

#if MJR_ROS_VERSION == ROS_2
		if (GetExecutorPtr() != nullptr) {
			GetExecutorPtr()->cancel();
		}
		if (executor_thread_handle_.joinable()) {
			executor_thread_handle_.join();
		}
#endif
	}

	std::string GetHandleNamespace()
	{
#if MJR_ROS_VERSION == ROS_1
		return nh_->getNamespace();
#else // MJR_ROS_VERSION == ROS_2
		return std::string(get_namespace()) + std::string(get_name());
#endif
	}

	void StartWithXML(const std::string &xml_path, bool wait = true, float timeout_secs = 2.)
	{
		mju::strcpy_arr(queued_filename_, xml_path.c_str());
		requestLoad(2);
		MujocoEnv::StartPhysicsLoop();
		MujocoEnv::StartEventLoop();

		if (not wait)
			return;

		// Wait for model to be loaded
		float seconds = 0;
		while (GetOperationalStatus() != 0 && seconds < timeout_secs) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
	}

#if MJR_ROS_VERSION == ROS_2
	std::thread executor_thread_handle_;
#endif
};

class BaseEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr = nullptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", true);
		nh->setParam("no_render", true);
		nh->setParam("headless", true);
		nh->setParam("render_offscreen", true);
		nh->setParam("use_sim_time", true);
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
#if MJR_ROS_VERSION == ROS_1
		// clean up all parameters
		ros::param::del(nh->getNamespace());
#else // MJR_ROS_VERSION == ROS_2
		nh->clearNode(); // Clear node reference
#endif
	}
};

class PendulumEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr = nullptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("sim_steps", -1);

		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

		std::string xml_path = testing::get_test_model_path("pendulum_world.xml");
		env_ptr->StartWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		ASSERT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";

		// Make sure forward has been run at least once
		{
			std::lock_guard<MujocoEnvMutex> lock(*env_ptr->getMutexPtr());
			mj_forward(env_ptr->getModelPtr(), env_ptr->getDataPtr());
		}
	}

	void TearDown() override
	{
		env_ptr->shutdown();
#if MJR_ROS_VERSION == ROS_1
		// clean up all parameters
		ros::param::del(nh->getNamespace());
#else // MJR_ROS_VERSION == ROS_2
		nh->clearNode(); // Clear node reference
#endif
	}
};

class EqualityEnvFixture : public ::testing::Test
{
protected:
	std::shared_ptr<testing::TestNodeHandle> nh;
	std::unique_ptr<MujocoEnvTestWrapper> env_ptr;
	mjModel *m;
	mjData *d;

	void SetUp() override
	{
		nh = std::make_shared<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("sim_steps", -1);

		// verify expected parameter array sizes
		EXPECT_EQ(mjNEQDATA, 11) << "This versions expects the maximum equality contraint parameters to be 11";
		EXPECT_EQ(mjNIMP, 5) << "This version expects the number of solimp parameters to be 5";
		EXPECT_EQ(mjNREF, 2) << "This version expects the number of solref parameters to be 2";

		// verify enum consistency
		EXPECT_EQ(mjEQ_CONNECT, EqualityConstraintType::CONNECT) << "Mismatch between connect constraint types";
		EXPECT_EQ(mjEQ_WELD, EqualityConstraintType::WELD) << "Mismatch between weld constraint types";
		EXPECT_EQ(mjEQ_JOINT, EqualityConstraintType::JOINT) << "Mismatch between joint constraint types";
		EXPECT_EQ(mjEQ_TENDON, EqualityConstraintType::TENDON) << "Mismatch between tendon constraint types";

		env_ptr = std::make_unique<MujocoEnvTestWrapper>("", nh.get());

		std::string xml_path = testing::get_test_model_path("equality_world.xml");
		env_ptr->StartWithXML(xml_path);

		float seconds = 0;
		while (env_ptr->GetOperationalStatus() != 0 && seconds < 2) { // wait for model to be loaded or timeout
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			seconds += 0.001;
		}
		EXPECT_EQ(env_ptr->getFilename(), xml_path) << "Model was not loaded correctly!";
		m = env_ptr->getModelPtr();
		d = env_ptr->getDataPtr();

		EXPECT_FALSE(env_ptr->GetControlSnapshot().running) << "Simulation should be paused!";
		EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
		EXPECT_TRUE(testing::wait_for_service(env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters"))
		    << "Set eq constraints service should be available!";
	}

	void TearDown() override { env_ptr->shutdown(); }
};
