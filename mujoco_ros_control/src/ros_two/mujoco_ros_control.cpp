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

#include "mujoco_ros_control/ros_two/mujoco_ros_control.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include <mujoco_ros/logging.hpp>
#include <chrono>
#include <future>
#include <set>

namespace mujoco_ros {
namespace control {

std::string concatenateNamespace(const std::string ns1, const std::string ns2)
{
	if (ns1.back() == '/') {
		return ns1 + ns2;
	} else {
		return ns1 + '/' + ns2;
	}
}
std::string MujocoRosControlPluginPrivate::getURDF() const
{
	std::string urdf_string;

	using namespace std::chrono_literals;
	auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_description_node_);
	while (!parameters_client->wait_for_service(0.5s)) {
		if (!rclcpp::ok()) {
			MJR_ERROR_NAMED("mujoco_ros_control", "Interrupted while waiting for %s service. Exiting.",
			                robot_description_node_.c_str());
			return {};
		}
		MJR_ERROR_NAMED("mujoco_ros_control", "%s service not available, waiting again...",
		                robot_description_node_.c_str());
	}

	MJR_INFO_NAMED("mujoco_ros_control", "connected to service. %s asking for %s", robot_description_node_.c_str(),
	               this->robot_description_.c_str());

	// search and wait for robot_description on param server
	while (urdf_string.empty()) {
		try {
			auto f = parameters_client->get_parameters({ this->robot_description_ });
			// Let the background executor handle the communication.
			auto status = f.wait_for(std::chrono::seconds(5));
			if (status == std::future_status::ready) {
				std::vector<rclcpp::Parameter> values = f.get();
				urdf_string                           = values[0].as_string();
			} else {
				MJR_ERROR_NAMED("mujoco_ros_control", "Service 'robot_description' timed out. "
				                                      "Ensure robot_state_publisher is running.");
			}
		} catch (const std::exception &e) {
			MJR_ERROR_NAMED("mujoco_ros_control", "%s", e.what());
		}

		if (!urdf_string.empty()) {
			break;
		} else {
			MJR_ERROR_NAMED("mujoco_ros_control",
			                "mujoco_ros_control plugin is waiting for model"
			                " URDF in parameter [%s] on the ROS param server.",
			                this->robot_description_.c_str());
		}
		std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
	MJR_INFO_NAMED("mujoco_ros_control", "Received URDF from param server");

	return urdf_string;
}

// MujocoRosControlPlugin::MujocoRosControlPlugin()
// {

// }

MujocoRosControlPlugin::~MujocoRosControlPlugin()
{
	// Stop controller manager thread
	if (!this->dataPtr_->controller_manager_) {
		return;
	}
	env_ptr_->RemoveNodeFromExecutor(this->dataPtr_->controller_manager_->get_node_base_interface());
	// this->dataPtr_->executor_->cancel();
	// this->dataPtr_->thread_executor_spin_.join();
}

mujoco_ros::CallbackReturn MujocoRosControlPlugin::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
	MJR_INFO_STREAM_NAMED("mujoco_ros_control", "Configuring mujoco_ros_control plugin");
	mujoco_ros::declare_parameter_if_not_declared(this->get_node()->get_node_parameters_interface(), "namespace",
	                                              rclcpp::ParameterValue(""));
	mujoco_ros::declare_parameter_if_not_declared(this->get_node()->get_node_parameters_interface(),
	                                              "robot_description_node",
	                                              rclcpp::ParameterValue("robot_state_publisher"));
	mujoco_ros::declare_parameter_if_not_declared(this->get_node()->get_node_parameters_interface(), "robot_description",
	                                              rclcpp::ParameterValue("robot_description"));

	return mujoco_ros::CallbackReturn::SUCCESS;
}

void MujocoRosControlPlugin::ControlCallback(const mjModel * /* model */, mjData *data)
{
	rclcpp::Time sim_time_mj    = rclcpp::Time(static_cast<int64_t>(data->time * 1e9), RCL_STEADY_TIME);
	rclcpp::Duration sim_period = sim_time_mj - this->dataPtr_->last_update_sim_time_mj_;
	this->dataPtr_->controller_manager_->write(sim_time_mj, sim_period);
};
void MujocoRosControlPlugin::PassiveCallback(const mjModel * /* model */, mjData *data)
{
	rclcpp::Time sim_time_mj                 = rclcpp::Time(static_cast<int64_t>(data->time * 1e9), RCL_STEADY_TIME);
	rclcpp::Duration sim_period              = sim_time_mj - this->dataPtr_->last_update_sim_time_mj_;
	this->dataPtr_->last_update_sim_time_mj_ = sim_time_mj;
	this->dataPtr_->controller_manager_->read(sim_time_mj, sim_period);
	this->dataPtr_->controller_manager_->update(sim_time_mj, sim_period);
};
void MujocoRosControlPlugin::RenderCallback(const mjModel * /* model */, mjData * /* data */, mjvScene * /* scene */) {
};
void MujocoRosControlPlugin::LastStageCallback(const mjModel * /* model */, mjData * /* data */) {};
void MujocoRosControlPlugin::OnGeomChanged(const mjModel * /* model */, mjData * /* data */, const int /* geom_id */) {
};

bool MujocoRosControlPlugin::Load(const mjModel *model, mjData *data)
{
	dataPtr_        = std::make_unique<MujocoRosControlPluginPrivate>();
	dataPtr_->node_ = get_node();
	MJR_INFO_STREAM_NAMED("mujoco_ros_control",
	                      "Namespace: " << get_node()->get_namespace() << "; Fully qualified name: "
	                                    << get_node()->get_node_base_interface()->get_fully_qualified_name());

	MJR_INFO_STREAM_NAMED("mujoco_ros_control", "loading with given model and data");
	// TODO: Need to rearrange the params to come from own yaml tree, along with on_configure settings

	// get the name of the robot_state_publisher node
	this->dataPtr_->robot_description_node_ = dataPtr_->node_->get_parameter("robot_description_node").as_string();
	MJR_INFO_STREAM_NAMED("mujoco_ros_control",
	                      "robot_description_node name is " << this->dataPtr_->robot_description_node_);

	// get the name of the srv from the node above that holds the URDF string
	// get the name of the robot_state_publisher node
	this->dataPtr_->robot_description_ = dataPtr_->node_->get_parameter("robot_description").as_string();
	MJR_INFO_STREAM_NAMED("mujoco_ros_control",
	                      "robot_description service name is " << this->dataPtr_->robot_description_);
	// todo: Make the logic for passing additional ros-args when initializing, like L292~ in ign_ros2_control_plugin.cpp

	// Construct the fully qualified robot_description_node's name.
	// The namespace will also be used for the nodes of this.
	std::string ns = dataPtr_->node_->get_parameter("namespace").as_string();

	// prevent exception: namespace must be absolute, it must lead with a '/'
	if (ns.empty() || ns[0] != '/') {
		ns = '/' + ns;
	}
	if (ns.length() > 1) {
		this->dataPtr_->robot_description_node_ = ns + "/" + this->dataPtr_->robot_description_node_;
	} else {
		this->dataPtr_->robot_description_node_ = ns + this->dataPtr_->robot_description_node_;
	}
	MJR_INFO_NAMED("mujoco_ros_control", "robot_description_node fully qualified name: %s",
	               this->dataPtr_->robot_description_node_.c_str());

	// Create a default context, if not already
	if (!rclcpp::ok()) {
		std::vector<const char *> _argv;
		rclcpp::init(static_cast<int>(_argv.size()),
		             _argv.data()); // todo: make the logic for passing additional ros-args
	}

	MJR_INFO_STREAM_NAMED("mujoco_ros_control",
	                      "Fully qualified mujoco_ros_control node name: "
	                          << this->dataPtr_->node_->get_node_base_interface()->get_fully_qualified_name());

	// executor creation
	this->dataPtr_->executor_ = env_ptr_->GetExecutorPtr();
	// Read urdf from ros parameter server then
	// setup actuators and mechanism control node.
	// This call will block if ROS is not properly initialized.
	std::string urdf_string;
	std::vector<hardware_interface::HardwareInfo> control_hardware_info;
	try {
		urdf_string           = this->dataPtr_->getURDF();
		control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
	} catch (const std::runtime_error &ex) {
		MJR_ERROR_STREAM_NAMED("mujoco_ros_control",
		                       "Error parsing URDF in mujoco_ros_control plugin, plugin not active : " << ex.what());
		return false;
	}

	std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
	    std::make_unique<hardware_interface::ResourceManager>();

	try {
		resource_manager_->load_urdf(urdf_string, false, false);
	} catch (...) {
		MJR_ERROR_NAMED("mujoco_ros_control", "Error initializing URDF to resource manager!");
	}
	try {
		this->dataPtr_->robot_hw_sim_loader_.reset(
		    new pluginlib::ClassLoader<mujoco_ros::control::MujocoRosSystemInterface>(
		        "mujoco_ros_control", "mujoco_ros::control::MujocoRosSystemInterface"));
	} catch (pluginlib::LibraryLoadException &ex) {
		MJR_ERROR_NAMED("mujoco_ros_control", "Failed to create robot simulation interface loader: %s ", ex.what());
		return false;
	}
	for (unsigned int i = 0; i < control_hardware_info.size(); ++i) {
		std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_class_type;
		std::unique_ptr<mujoco_ros::control::MujocoRosSystemInterface> mujocoRos2System;
		MJR_DEBUG_NAMED("mujoco_ros_control", "Load hardware interface %s ...", robot_hw_sim_type_str_.c_str());
		try {
			mujocoRos2System = std::unique_ptr<mujoco_ros::control::MujocoRosSystemInterface>(
			    this->dataPtr_->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
			MJR_INFO_NAMED("mujoco_ros_control", "createUnmanagedInstance");
		} catch (pluginlib::PluginlibException &ex) {
			MJR_ERROR_NAMED("mujoco_ros_control", "The plugin failed to load for some reason. Error: %s\n", ex.what());
			continue;
		}
		if (!mujocoRos2System->initSim(this->dataPtr_->node_, control_hardware_info[i], model, data,
		                               this->dataPtr_->update_rate)) {
			MJR_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot simulation interface");
			return false;
		}
		MJR_DEBUG_NAMED("mujoco_ros_control", "Initialized robot simulation interface %s!",
		                robot_hw_sim_type_str_.c_str());

		MJR_DEBUG_STREAM_NAMED("mujoco_ros_control",
		                       "resource-manager system comp size: " << resource_manager_->system_components_size());
		resource_manager_->import_component(std::move(mujocoRos2System), control_hardware_info[i]);

		MJR_DEBUG_STREAM_NAMED("mujoco_ros_control",
		                       "resource-manager system comp size: " << resource_manager_->system_components_size());
		MJR_DEBUG_NAMED("mujoco_ros_control", "Setting state of %s to active", control_hardware_info[i].name.c_str());
		rclcpp_lifecycle::State state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
		resource_manager_->set_component_state(control_hardware_info[i].name, state);
	}

	// Create the controller manager

	// Get controller manager node name
	std::string controllerManagerNodeName{ "controller_manager" };
	MJR_INFO_NAMED("mujoco_ros_control", "Loading controller_manager");
	// The node options are very specific. The two function(true) lines mimic what is set for
	// controller_manager::get_cm_options(); without them, the controller manager will break.
	auto cm_options = rclcpp::NodeOptions().arguments(
	    { "--ros-args", "--remap", controllerManagerNodeName + ":__node:=" + controllerManagerNodeName, "--remap",
	      "joint_state_broadcaster:__node:=joint_state_broadcaster_alt" });
	cm_options.allow_undeclared_parameters(true);
	cm_options.automatically_declare_parameters_from_overrides(true);

	// This part sometimes breaks with error munmap_chunk(): invalid pointer
	this->dataPtr_->controller_manager_.reset(new controller_manager::ControllerManager(
	    std::move(resource_manager_), this->dataPtr_->executor_, controllerManagerNodeName, ns, cm_options));
	env_ptr_->AddNodeToExecutor(this->dataPtr_->controller_manager_->get_node_base_interface());
	MJR_WARN_STREAM_NAMED("mujoco_ros_control", this->dataPtr_->controller_manager_->get_fully_qualified_name()
	                                                << ns << controllerManagerNodeName);
	if (!this->dataPtr_->controller_manager_->has_parameter("update_rate")) {
		MJR_ERROR_STREAM_NAMED("mujoco_ros_control", "controller manager doesn't have an update_rate parameter");
		return false;
	}

	this->dataPtr_->update_rate     = this->dataPtr_->controller_manager_->get_update_rate();
	this->dataPtr_->control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
	    std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr_->update_rate))));

	// Force setting of use_sim_time parameter
	this->dataPtr_->controller_manager_->set_parameter(rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

	return true;
}

void MujocoRosControlPlugin::Reset()
{
	MJR_INFO_STREAM_NAMED("mujoco_ros_control", "reset");
}

} // namespace control
} // namespace mujoco_ros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros::control::MujocoRosControlPlugin, mujoco_ros::MujocoPlugin)
