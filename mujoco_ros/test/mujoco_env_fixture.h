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

/* Authors: David P. Leins */

#include <gtest/gtest.h>

#include <atomic>
#include <map>
#include <mutex>
#include <vector>

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

namespace service {
inline bool exists(const std::string & /*service_name*/, bool /*print_failure_reason*/)
{
	return true;
}

template <typename ServiceType>
bool call(const std::string & /*service_name*/, ServiceType & /*srv*/)
{
	return true;
}
} // namespace service

namespace param {
inline void del(const std::string & /*param_name*/) {}
} // namespace param
} // namespace ros

#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mujoco_ros_test"), __VA_ARGS__)
#endif
#endif

#include <mujoco_ros/mujoco_env.hpp>

using namespace mujoco_ros;
namespace mju = ::mujoco::sample_util;

namespace testing {

#if MJR_ROS_VERSION == ROS_1
using TestNodeHandle = ros::NodeHandle;

inline void delete_namespace_params(const std::string &ns)
{
	ros::param::del(ns);
}
#else // MJR_ROS_VERSION == ROS_2
class TestNodeHandle
{
public:
	explicit TestNodeHandle(const std::string & /*ns*/) {}

	template <typename T>
	void setParam(const std::string &name, const T &value)
	{
		std::string name_copy = name;
		std::replace(name_copy.begin(), name_copy.end(), '/',
		             '.'); // ROS 2 parameters use '.' instead of '/' as nesting does not exist in the same way as ROS 1
		std::lock_guard<std::mutex> lock(param_mutex_);
		pending_params_[name_copy] = rclcpp::Parameter(name_copy, value);
	}

	template <typename T>
	bool getParam(const std::string &name, T &value) const
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		auto it = pending_params_.find(name);
		if (it == pending_params_.end()) {
			return false;
		}
		try {
			value = it->second.get_value<T>();
			return true;
		} catch (...) {
			return false;
		}
	}

	std::string getNamespace() const { return "/mujoco_server"; }

	static std::vector<rclcpp::Parameter> GetPendingParams()
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		std::vector<rclcpp::Parameter> params;
		params.reserve(pending_params_.size());
		for (const auto &entry : pending_params_) {
			params.push_back(entry.second);
		}
		return params;
	}

	static void clearPendingParams()
	{
		std::lock_guard<std::mutex> lock(param_mutex_);
		pending_params_.clear();
	}

private:
	static std::mutex param_mutex_;
	static std::map<std::string, rclcpp::Parameter> pending_params_;
};

inline std::mutex TestNodeHandle::param_mutex_;
inline std::map<std::string, rclcpp::Parameter> TestNodeHandle::pending_params_;

inline void delete_namespace_params(const std::string & /*ns*/)
{
	TestNodeHandle::clearPendingParams();
}
#endif

inline std::string get_test_model_path(const std::string &model_name)
{
#if MJR_ROS_VERSION == ROS_1
	return ros::package::getPath("mujoco_ros") + "/test/" + model_name;
#else // MJR_ROS_VERSION == ROS_2
	return ament_index_cpp::get_package_share_directory("mujoco_ros") + "/test/" + model_name;
#endif
}

// Helper struct for topic info abstraction
struct TopicInfo
{
	std::string name;
};

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

#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
inline bool wait_for_service(const std::string &service_name)
{
#if MJR_ROS_VERSION == ROS_1
	return ros::service::exists(service_name, true);
#else // MJR_ROS_VERSION == ROS_2
	return true;
#endif
}
#pragma diagnostic pop

} // namespace testing

class MujocoEnvTestWrapper : public MujocoEnv
{
public:
#if MJR_ROS_VERSION == ROS_1
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string())
	    : MujocoEnv(admin_hash), construction_complete_(true)
	{
	}
#else // MJR_ROS_VERSION == ROS_2
	MujocoEnvTestWrapper(const std::string &admin_hash = std::string())
	    : MujocoEnv(std::make_shared<rclcpp::executors::MultiThreadedExecutor>(), admin_hash, false)
	    , construction_complete_(false)
	{
		const auto params = testing::TestNodeHandle::GetPendingParams();
		for (const auto &param : params) {
			if (!this->has_parameter(param.get_name())) {
				this->declare_parameter(param.get_name(), param.get_parameter_value());
			}
		}
		this->set_parameters(params);
		GetExecutorPtr()->add_node(this->get_node_base_interface());

		// Start executor thread BEFORE Configure() so services and callbacks have a spinning executor
		executor_thread_handle_ = std::thread([this]() { GetExecutorPtr()->spin(); });

		// Give executor thread a moment to start spinning
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		try {
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

	~MujocoEnvTestWrapper() override
	{
		try {
			shutdown();
		} catch (const std::exception &e) {
			MJR_ERROR_STREAM("Exception during shutdown in destructor: " << e.what());
		} catch (...) {
			MJR_ERROR("Unknown exception during shutdown in destructor");
		}
	}

#endif
	std::atomic_bool shutdown_called_{ false };
	std::atomic_bool construction_complete_{ false };
	mjModel *getModelPtr()
	{
		return model_.get();
	}
	mjData *getDataPtr()
	{
		return data_.get();
	}
	MujocoEnvMutex *getMutexPtr()
	{
		return &physics_thread_mutex_;
	}

#if MJR_ROS_VERSION == ROS_1
	dynamic_reconfigure::Server<mujoco_ros::SimParamsConfig> *getParamServer()
	{
		return nullptr;
	}
#else // MJR_ROS_VERSION == ROS_2
	void *getParamServer()
	{
		return nullptr;
	}
#endif

	int getPendingSteps()
	{
		return num_steps_until_exit_;
	}

	void setEvalMode(bool eval_mode)
	{
		settings_.eval_mode = eval_mode;
	}
	void setAdminHash(const std::string &hash)
	{
		mju::strcpy_arr(settings_.admin_hash, hash.c_str());
	}

	std::string getFilename()
	{
		return { filename_ };
	}
	int isPhysicsRunning()
	{
		return is_physics_running_;
	}
	int isEventRunning()
	{
		return is_event_running_;
	}
	int isRenderingRunning()
	{
		return is_rendering_running_;
	}

	OffscreenRenderContext *getOffscreenContext()
	{
		return &offscreen_;
	}

	int GetNumCBReadyPlugins()
	{
		return cb_ready_plugins_.size();
	}
	void NotifyGeomChange()
	{
		NotifyGeomChanged(0);
	}

	bool step(int num_steps = 1, bool blocking = true)
	{
		return MujocoEnv::Step(num_steps, blocking);
	}
	bool togglePaused(bool paused, const std::string &admin_hash = std::string())
	{
		return TogglePaused(paused, admin_hash);
	}
	int GetOperationalStatus()
	{
		return MujocoEnv::GetOperationalStatus();
	}
	void StartPhysicsLoop()
	{
		MujocoEnv::StartPhysicsLoop();
	}
	void StartEventLoop()
	{
		MujocoEnv::StartEventLoop();
	}
	void WaitForPhysicsJoin()
	{
		MujocoEnv::WaitForPhysicsJoin();
	}
	void WaitForEventsJoin()
	{
		MujocoEnv::WaitForEventsJoin();
	}

	void load_queued_model()
	{
		settings_.load_request = 2;
		float seconds          = 0;
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
			settings_.exit_request = 1;
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
		settings_.load_request = 2;
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
		nh->setParam("use_sim_time", true);
	}

	void TearDown() override
	{
		if (env_ptr != nullptr) {
			env_ptr->shutdown();
		}
		// clean up all parameters
		testing::delete_namespace_params(nh->getNamespace());
	}
};

class PendulumEnvFixture : public ::testing::Test
{
protected:
	std::unique_ptr<testing::TestNodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;

	void SetUp() override
	{
		nh = std::make_unique<testing::TestNodeHandle>("~");
		nh->setParam("unpause", false);
		nh->setParam("no_render", true);
		nh->setParam("use_sim_time", true);
		nh->setParam("sim_steps", -1);

		env_ptr = new MujocoEnvTestWrapper();

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
		delete env_ptr;
	}
};

class EqualityEnvFixture : public ::testing::Test
{
protected:
	std::shared_ptr<testing::TestNodeHandle> nh;
	MujocoEnvTestWrapper *env_ptr;
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

		env_ptr = new MujocoEnvTestWrapper();

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

		EXPECT_FALSE(env_ptr->settings_.run) << "Simulation should be paused!";
		EXPECT_NEAR(d->time, 0, 1e-6) << "Simulation time should be 0.0!";
		EXPECT_TRUE(testing::wait_for_service(env_ptr->GetHandleNamespace() + "/set_eq_constraint_parameters"))
		    << "Set eq constraints service should be available!";
	}

	void TearDown() override
	{
		env_ptr->shutdown();
		delete env_ptr;
	}
};
