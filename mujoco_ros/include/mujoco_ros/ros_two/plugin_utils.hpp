/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/mujoco_env.hpp>

#include <pluginlib/class_loader.hpp>

namespace mujoco_ros {
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MujocoPlugin : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
	/**
	 * Override default implementation for configure to get parameters.
	 */
	const rclcpp_lifecycle::State &configure();

	virtual ~MujocoPlugin()
	{
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MujocoPlugin"), "Deleted plugin of type " << type_);
		if (node_.get() && get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED &&
		    rclcpp::ok()) {
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger(plugin_name_), "Removing node from executor");
			node_->shutdown();
			env_ptr_->RemoveNodeFromExecutor(this->get_node()->get_node_base_interface());
		}
	}

	// Called directly after plugin creation
	void Init(const std::string &name, MujocoEnv *env_ptr, const std::string &type)
	{
		env_ptr_     = env_ptr;
		plugin_name_ = name;
		type_        = type;
		// std::string ns(env_ptr->get_name());
		// ns.append("/"+name);
		// RCLCPP_ERROR_STREAM(rclcpp::get_logger("MujocoPlugin"), "Namespace: " << ns << "(" << env_ptr->get_name() << "
		// + /" << name << ")");
		auto options          = rclcpp::NodeOptions();
		std::string remap_str = name + ":__node:=" + name;
		options.arguments({ "--ros-args", "--remap", remap_str });
		node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(name, options, false);

		node_->register_on_configure(std::bind(&MujocoPlugin::on_configure, this, std::placeholders::_1));
		node_->register_on_cleanup(std::bind(&MujocoPlugin::on_cleanup, this, std::placeholders::_1));
		node_->register_on_activate(std::bind(&MujocoPlugin::on_activate, this, std::placeholders::_1));
		node_->register_on_deactivate(std::bind(&MujocoPlugin::on_deactivate, this, std::placeholders::_1));
		node_->register_on_shutdown(std::bind(&MujocoPlugin::on_shutdown, this, std::placeholders::_1));
		node_->register_on_error(std::bind(&MujocoPlugin::on_error, this, std::placeholders::_1));
	};

	const rclcpp_lifecycle::State &get_lifecycle_state() const
	{
		if (!node_.get()) {
			throw std::runtime_error("Node has not been initialized yet. Call Init() on plugins first");
		}
		return node_->get_current_state();
	}

	std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> get_node() const
	{
		if (!node_.get()) {
			throw std::runtime_error("Node has not been initialized yet. Call Init() on plugins first");
		}
		return node_;
	}
	std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node()
	{
		if (!node_.get()) {
			throw std::runtime_error("Node has not been initialized yet. Call Init() on plugins first");
		}
		return node_;
	}

	std::string get_name() const { return plugin_name_; }
	std::string get_type() const { return type_; }
	double get_load_time() const { return load_time_; }
	double get_reset_time() const { return reset_time_; }
	double get_ema_steptime_control() const { return ema_steptime_control_; }
	double get_ema_steptime_passive() const { return ema_steptime_passive_; }
	double get_ema_steptime_render() const { return ema_steptime_render_; }
	double get_ema_steptime_last_stage() const { return ema_steptime_last_stage_; }

	/**
	 * @brief Wrapper method that evaluates if loading the plugin is successful
	 *
	 * @param[in] m
	 * @param[out] d
	 * @return true if plugin could be loaded without errors.
	 * @return false if errors occurred during loading.
	 */
	bool SafeLoad(const mjModel *m, mjData *d)
	{
		const auto start    = Clock::now();
		loading_successful_ = Load(m, d);
		load_time_          = Seconds(Clock::now() - start).count();
		if (!loading_successful_)
			RCLCPP_WARN_STREAM(rclcpp::get_logger("MujocoPlugin"),
			                   "Plugin '" << plugin_name_ << "' of type '" << type_
			                              << "' failed to load. It will be ignored until the next load attempt.");
		return loading_successful_;
	}

	/**
	 * @brief Wrapper method that only calls reset if loading the plugin was successful.
	 */
	void SafeReset()
	{
		if (loading_successful_) {
			const auto start = Clock::now();
			Reset();
			reset_time_ = Seconds(Clock::now() - start).count();
		}
	}

	/**
	 * @brief Wrapper method that calls controlCallback and counts the time for the exponential moving average.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	void WrappedControlCallback(const mjModel *model, mjData *data)
	{
		const auto start = Clock::now();
		skip_ema_        = false;
		ControlCallback(model, data);
		if (skip_ema_) {
			return;
		}
		const auto elapsed_secs = Seconds(Clock::now() - start).count();
		// update ema with sensitivity for ~1000 steps
		ema_steptime_control_ = 0.002 * elapsed_secs + 0.998 * ema_steptime_control_;
	}

	/**
	 * @brief Wrapper method that calls passiveCallback and counts the time for the exponential moving average.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	void WrappedPassiveCallback(const mjModel *model, mjData *data)
	{
		const auto start = Clock::now();
		skip_ema_        = false;
		PassiveCallback(model, data);
		if (skip_ema_) {
			return;
		}
		const auto elapsed_secs = Seconds(Clock::now() - start).count();
		// update ema with sensitivity for ~1000 steps
		ema_steptime_passive_ = 0.002 * elapsed_secs + 0.998 * ema_steptime_passive_;
	}

	/**
	 * @brief Wrapper method that calls renderCallback and counts the time for the exponential moving average.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] scene pointer to mjvScene.
	 */
	void WrappedRenderCallback(const mjModel *model, mjData *data, mjvScene *scene)
	{
		const auto start = Clock::now();
		skip_ema_        = false;
		RenderCallback(model, data, scene);
		if (skip_ema_) {
			return;
		}
		const auto elapsed_secs = Seconds(Clock::now() - start).count();
		// update ema with sensitivity for ~1000 steps
		ema_steptime_render_ = 0.002 * elapsed_secs + 0.998 * ema_steptime_render_;
	}

	/**
	 * @brief Wrapper method that calls lastStageCallback and counts the time for the exponential moving average.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	void WrappedLastStageCallback(const mjModel *model, mjData *data)
	{
		const auto start = Clock::now();
		skip_ema_        = false;
		LastStageCallback(model, data);
		if (skip_ema_) {
			return;
		}
		const auto elapsed_secs = Seconds(Clock::now() - start).count();
		// update ema with sensitivity for ~1000 steps
		ema_steptime_last_stage_ = 0.002 * elapsed_secs + 0.998 * ema_steptime_last_stage_;
	}

	/**
	 * @brief Override this callback to add custom behavior when a geom has been changed in the model.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] geom_id id of the geom thas has been changed.
	 */
	virtual void OnGeomChanged(const mjModel * /*model*/, mjData * /*data*/, const int /*geom_id*/){};

protected:
	/**
	 * @brief Override this function to implement custom control laws.
	 * To apply control, write into \c mjData.ctrl, \c mjData.qfrc_applied and/or \c mjData.xfrc_applied.
	 * If defined, this function will be called by the mujoco step function at the appropriate time.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void ControlCallback(const mjModel * /*model*/, mjData * /*data*/) { skip_ema_ = true; };

	/**
	 * @brief Override this function to compute and apply custom passive (i.e. non-controlled) forces.
	 * This callback should add to the vector \c mjData.qfrc_passive instead of overwriting it, otherwise
	 * the standard passive forces will be lost.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void PassiveCallback(const mjModel * /*model*/, mjData * /*data*/) { skip_ema_ = true; };

	/**
	 * @brief Override this callback to add custom visualisations to the scene.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 * @param[in] scene pointer to mjvScene.
	 */
	virtual void RenderCallback(const mjModel * /*model*/, mjData * /*data*/, mjvScene * /*scene*/)
	{
		skip_ema_ = true;
	};

	/**
	 * @brief Override this callback to add custom behavior at the end of a mujoco_ros simulation step.
	 * Note that unlike `controlCallback` and `passiveCallback` this callback will not be called when mujoco runs
	 * individual sub-steps.
	 *
	 * @param[in] model pointer to const mjModel.
	 * @param[in] data pointer to mjData.
	 */
	virtual void LastStageCallback(const mjModel * /*model*/, mjData * /*data*/) { skip_ema_ = true; };

	/**
	 * @brief Called once the world is loaded.
	 *
	 * @param[in] m shared pointer to mujoco model.
	 * @param[in] d shared pointer to mujoco data.
	 * @return true on succesful load.
	 * @return false if load was not successful.
	 */
	virtual bool Load(const mjModel *m, mjData *d) = 0;

	/**
	 * @brief Called on reset.
	 */
	virtual void Reset() = 0;

private:
	bool loading_successful_ = false;
	std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

protected:
	MujocoPlugin() = default;
	MujocoEnvPtr env_ptr_;
	// Plugin instance name
	std::string plugin_name_;
	// Plugin type as string
	std::string type_;
	// load time
	double load_time_ = -1.0;
	// reset time
	double reset_time_ = -1.0;
	// exponential moving average control step time in seconds
	double ema_steptime_control_ = 0.0;
	// exponential moving average passive step time in seconds
	double ema_steptime_passive_ = 0.0;
	// exponential moving average render step time in seconds
	double ema_steptime_render_ = 0.0;
	// exponential moving average last stage step time in seconds
	double ema_steptime_last_stage_ = 0.0;

	// flag to skip ema calculation (e.g. plugins calculation frequencies lower than sim step size)
	bool skip_ema_ = false;
};

namespace plugin_utils {

/**
 * @brief Searches for plugins to load in the nodes parameters and stores the names in \c plugin_names.
 * @param[in] env_ptr Pointer to the current MujocoEnv (inheriting from ros2 node) used to fetch plugin parameters.
 * @param[inout] plugin_names If any plugin names are configured to be loaded, they are stored in this variable.
 */
bool ParsePlugins(MujocoEnv *env_ptr, std::vector<std::string> &plugin_names);

/**
 * @brief Tries to load each plugin defined in \c plugin_names using the name as path to load its respective
 * configuration.
 *
 * @param[in] plugin_names nodehandle namespace.
 * @param[inout] plugins vector of plugins. If successfully initialized, each plugin is appended to the vector.
 */
void RegisterPlugins(const std::vector<std::string> &plugin_names, std::vector<MujocoPluginPtr> &plugins,
                     MujocoEnv *env_ptr);

void UnloadPluginloader();
void InitPluginLoader();

static std::unique_ptr<pluginlib::ClassLoader<MujocoPlugin>> plugin_loader_ptr_;

/**
 * @brief Defines under which path the plugin configuration is stored in the ros parameter server.
 */
const static std::string MUJOCO_PLUGIN_PARAM_NAME = "MujocoPlugins";

} // end namespace plugin_utils
} // namespace mujoco_ros
