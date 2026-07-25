/*********************************************************************
 * Software License Agreement (BSD License)
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

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/logging.hpp>

#include <mujoco_ros/array_safety.h>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_two/plugin_utils.hpp>
#include <mujoco_ros/ros_two/ros_api.hpp>
#include <mujoco_ros/util.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sstream>

namespace mujoco_ros {
namespace mju = ::mujoco::sample_util;

namespace {

thread_local bool syncing_dynamic_params = false;

rcl_interfaces::msg::ParameterDescriptor MakeParameterDescriptor(const std::string &description = "")
{
	rcl_interfaces::msg::ParameterDescriptor descriptor;
	descriptor.dynamic_typing = false;
	descriptor.description    = description;
	return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor MakeEnumDescriptor(const std::string &description, int64_t lower_bound,
                                                            int64_t upper_bound,
                                                            const std::string &additional_constraints)
{
	auto descriptor = MakeParameterDescriptor(description);
	rcl_interfaces::msg::IntegerRange range;
	range.from_value = lower_bound;
	range.to_value   = upper_bound;
	range.step       = 1;
	descriptor.integer_range.emplace_back(range);
	descriptor.additional_constraints = additional_constraints;
	return descriptor;
}

template <typename T>
void DeclareRuntimeParameter(MujocoEnvPtr env_ptr, const std::string &name, const T &value,
                             const std::string &description = "")
{
	declare_parameter_if_not_declared(env_ptr, name, rclcpp::ParameterValue(value),
	                                  MakeParameterDescriptor(description));
}

template <typename T>
void DeclareRuntimeParameter(MujocoEnvPtr env_ptr, const std::string &name, const T &value,
                             const rcl_interfaces::msg::ParameterDescriptor &descriptor)
{
	declare_parameter_if_not_declared(env_ptr, name, rclcpp::ParameterValue(value), descriptor);
}

class ScopedBoolFlag
{
public:
	explicit ScopedBoolFlag(bool &flag) : flag_(flag) { flag_ = true; }
	~ScopedBoolFlag() { flag_ = false; }

private:
	bool &flag_;
};

void ValidateIntegerRange(const rclcpp::Parameter &parameter, int64_t lower_bound, int64_t upper_bound)
{
	if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
		throw std::runtime_error("'" + parameter.get_name() + "' must be an integer parameter");
	}

	const auto value = parameter.as_int();
	if (value < lower_bound || value > upper_bound) {
		throw std::runtime_error("'" + parameter.get_name() + "' must be between " + std::to_string(lower_bound) +
		                         " and " + std::to_string(upper_bound));
	}
}

std::string ArrayToString(const mjtNum *array, int size)
{
	std::string value;
	util::arr_to_string(array, size, value);
	return value;
}

bool ParseMjtNumArray(const std::string &value, mjtNum *array, uint8_t size, std::string &reason)
{
	std::istringstream stream(value);
	std::string token;
	uint8_t count = 0;
	std::vector<mjtNum> parsed_values(size, 0);
	while (stream >> token) {
		if (count >= size) {
			MJR_WARN_STREAM("Too many values in string '" << value << "' expected " << static_cast<int>(size)
			                                              << ". Ignoring the rest.");
			continue;
		}

		try {
			size_t parsed        = 0;
			parsed_values[count] = std::stod(token, &parsed);
			if (parsed != token.size()) {
				reason = "Invalid numeric token '" + token + "' in '" + value + "'";
				return false;
			}
		} catch (const std::exception &) {
			reason = "Invalid numeric token '" + token + "' in '" + value + "'";
			return false;
		}
		++count;
	}

	if (count < size - 1) {
		MJR_WARN_STREAM("Too few values in string '" << value << "' expected " << static_cast<int>(size)
		                                             << ". Filling with zeros.");
		for (uint8_t i = count; i < size; i++) {
			parsed_values[i] = 0;
		}
	}
	for (uint8_t i = 0; i < size; ++i) {
		array[i] = parsed_values[i];
	}
	return true;
}

} // namespace

RosAPI::RosAPI(MujocoEnvPtr env_ptr) : env_ptr_(env_ptr)
{
	RCLCPP_DEBUG(env_ptr_->get_logger(), "Declaring default ros parameters");
	declare_parameter_if_not_declared(env_ptr_, "eval_mode", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "no_render", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "render_offscreen", rclcpp::ParameterValue(true));
	declare_parameter_if_not_declared(env_ptr_, "headless", rclcpp::ParameterValue(true));
	declare_parameter_if_not_declared(env_ptr_, "unpause", rclcpp::ParameterValue(true));
	declare_parameter_if_not_declared(env_ptr_, "num_steps", rclcpp::ParameterValue(-1));
	declare_parameter_if_not_declared(env_ptr_, "num_mj_threads", rclcpp::ParameterValue(1));
	declare_parameter_if_not_declared(env_ptr_, "modelfile", rclcpp::ParameterValue(std::string("")));
	declare_parameter_if_not_declared(env_ptr_, "realtime", rclcpp::ParameterValue(1.0));
	declare_parameter_if_not_declared(env_ptr_, plugin_utils::MUJOCO_PLUGIN_PARAM_NAME + ".names",
	                                  rclcpp::ParameterValue(std::vector<std::string>{}));
	declare_parameter_if_not_declared(env_ptr_, "wait_for_xml", rclcpp::ParameterValue(false));
	declare_parameter_if_not_declared(env_ptr_, "mujoco_xml", rclcpp::ParameterValue(std::string("")));
	declare_parameter_if_not_declared(env_ptr_, "use_sim_time", rclcpp::ParameterValue(true));

	DeclareRuntimeParameter(env_ptr_, "running", true, "Runtime pause state.");
	DeclareRuntimeParameter(env_ptr_, "admin_hash", std::string(""),
	                        "Admin hash used for protected runtime operations.");

	DeclareRuntimeParameter(
	    env_ptr_, "integrator", mjINT_EULER,
	    MakeEnumDescriptor("MuJoCo integrator.", 0, 3, "0: Euler, 1: RK4, 2: Implicit, 3: Implicitfast"));
	DeclareRuntimeParameter(env_ptr_, "cone", mjCONE_ELLIPTIC,
	                        MakeEnumDescriptor("MuJoCo cone type.", 0, 1, "0: Pyramidal, 1: Elliptic"));
	DeclareRuntimeParameter(env_ptr_, "jacobian", mjJAC_AUTO,
	                        MakeEnumDescriptor("MuJoCo Jacobian type.", 0, 2, "0: Dense, 1: Sparse, 2: Auto"));
	DeclareRuntimeParameter(env_ptr_, "solver", mjSOL_NEWTON,
	                        MakeEnumDescriptor("MuJoCo solver type.", 0, 2, "0: PGS, 1: CG, 2: Newton"));
	DeclareRuntimeParameter(env_ptr_, "timestep", 1e-3);
	DeclareRuntimeParameter(env_ptr_, "iterations", 100);
	DeclareRuntimeParameter(env_ptr_, "tolerance", 1e-8);
	DeclareRuntimeParameter(env_ptr_, "ls_iter", 50);
	DeclareRuntimeParameter(env_ptr_, "ls_tol", 0.01);
	DeclareRuntimeParameter(env_ptr_, "noslip_iter", 0);
	DeclareRuntimeParameter(env_ptr_, "noslip_tol", 1e-6);
	DeclareRuntimeParameter(env_ptr_, "ccd_iter", 50);
	DeclareRuntimeParameter(env_ptr_, "ccd_tol", 1e-6);
	DeclareRuntimeParameter(env_ptr_, "sdf_iter", 10);
	DeclareRuntimeParameter(env_ptr_, "sdf_init", 40);

	DeclareRuntimeParameter(env_ptr_, "gravity", std::string("0 0 -9.81"));
	DeclareRuntimeParameter(env_ptr_, "wind", std::string("0 0 0"));
	DeclareRuntimeParameter(env_ptr_, "magnetic", std::string("0 -0.5 0"));
	DeclareRuntimeParameter(env_ptr_, "density", 0.0);
	DeclareRuntimeParameter(env_ptr_, "viscosity", 0.0);
	DeclareRuntimeParameter(env_ptr_, "impratio", 1.0);

	DeclareRuntimeParameter(env_ptr_, "constraint_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "equality_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "frictionloss_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "limit_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "contact_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "passive_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "gravity_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "clampctrl_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "warmstart_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "filterparent_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "actuation_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "refsafe_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "sensor_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "midphase_disabled", false);
	DeclareRuntimeParameter(env_ptr_, "eulerdamp_disabled", false);

	DeclareRuntimeParameter(env_ptr_, "override_contacts", false);
	DeclareRuntimeParameter(env_ptr_, "energy", false);
	DeclareRuntimeParameter(env_ptr_, "fwd_inv", false);
	DeclareRuntimeParameter(env_ptr_, "inv_discrete", false);
	DeclareRuntimeParameter(env_ptr_, "multiccd", false);
	DeclareRuntimeParameter(env_ptr_, "island", false);

	DeclareRuntimeParameter(env_ptr_, "margin", 0.0);
	DeclareRuntimeParameter(env_ptr_, "solimp", std::string("0.9 0.95 0.00"));
	DeclareRuntimeParameter(env_ptr_, "solref", std::string("0.02 1.0"));
	DeclareRuntimeParameter(env_ptr_, "friction", std::string("1 1 0.05 0."));

	dynamic_params_callback_handle_ =
	    env_ptr_->add_on_set_parameters_callback(std::bind(&RosAPI::DynamicParamsCallback, this, std::placeholders::_1));
}

void RosAPI::SetupServices()
{
	std::string ns = std::string(env_ptr_->get_effective_namespace()) + "/" + std::string(env_ptr_->get_name());

	// Replace duplicate slashes in namespace
	std::string::size_type spos = 0;
	if ((spos = ns.find("//")) != std::string::npos) {
		MJR_WARN("Namespace contains duplicate slashes. Replacing '//' with '/'.");
		while ((spos = ns.find("//")) != std::string::npos) {
			ns.replace(spos, 2, "/");
		}
	}

	MJR_INFO_STREAM("Effective namespace: " << ns);

	// Add trailing slash if missing
	if (ns.back() != '/') {
		ns += '/';
	}

	set_pause_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetPause>(
	    ns + "set_pause", std::bind(&RosAPI::SetPauseCB, this, std::placeholders::_1, std::placeholders::_2));
	shutdown_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "shutdown", std::bind(&RosAPI::ShutdownCB, this, std::placeholders::_1, std::placeholders::_2));
	if (!env_ptr_->UsesPythonReloadService()) {
		reload_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::Reload>(
		    ns + "reload", std::bind(&RosAPI::ReloadCB, this, std::placeholders::_1, std::placeholders::_2));
	}
	reset_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "reset", std::bind(&RosAPI::ResetCB, this, std::placeholders::_1, std::placeholders::_2));
	set_body_state_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetBodyState>(
	    ns + "set_body_state", std::bind(&RosAPI::SetBodyStateCB, this, std::placeholders::_1, std::placeholders::_2));
	get_body_state_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetBodyState>(
	    ns + "get_body_state", std::bind(&RosAPI::GetBodyStateCB, this, std::placeholders::_1, std::placeholders::_2));
	set_geom_properties_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetGeomProperties>(
	    ns + "set_geom_properties",
	    std::bind(&RosAPI::SetGeomPropertiesCB, this, std::placeholders::_1, std::placeholders::_2));
	get_geom_properties_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetGeomProperties>(
	    ns + "get_geom_properties",
	    std::bind(&RosAPI::GetGeomPropertiesCB, this, std::placeholders::_1, std::placeholders::_2));
	set_eq_constraint_parameters_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetEqualityConstraintParameters>(
	    ns + "set_eq_constraint_parameters",
	    std::bind(&RosAPI::SetEqualityConstraintParametersArrayCB, this, std::placeholders::_1, std::placeholders::_2));
	get_eq_constraint_parameters_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetEqualityConstraintParameters>(
	    ns + "get_eq_constraint_parameters",
	    std::bind(&RosAPI::GetEqualityConstraintParametersArrayCB, this, std::placeholders::_1, std::placeholders::_2));
	load_initial_joint_states_srv_ = env_ptr_->create_service<std_srvs::srv::Empty>(
	    ns + "load_initial_joint_states",
	    std::bind(&RosAPI::LoadInitialJointStatesCB, this, std::placeholders::_1, std::placeholders::_2));
	get_state_uint_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetStateUint>(
	    ns + "get_loading_request_state",
	    std::bind(&RosAPI::GetStateUintCB, this, std::placeholders::_1, std::placeholders::_2));
	get_sim_info_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetSimInfo>(
	    ns + "get_sim_info", std::bind(&RosAPI::GetSimInfoCB, this, std::placeholders::_1, std::placeholders::_2));
	set_rt_factor_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetFloat>(
	    ns + "set_rt_factor", std::bind(&RosAPI::SetRTFactorCB, this, std::placeholders::_1, std::placeholders::_2));
	get_plugin_stats_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetPluginStats>(
	    ns + "get_plugin_stats",
	    std::bind(&RosAPI::GetPluginStatsCB, this, std::placeholders::_1, std::placeholders::_2));
	set_gravity_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::SetGravity>(
	    ns + "set_gravity", std::bind(&RosAPI::SetGravityCB, this, std::placeholders::_1, std::placeholders::_2));
	get_gravity_srv_ = env_ptr_->create_service<mujoco_ros_msgs::srv::GetGravity>(
	    ns + "get_gravity", std::bind(&RosAPI::GetGravityCB, this, std::placeholders::_1, std::placeholders::_2));

	action_step_ = rclcpp_action::create_server<mujoco_ros_msgs::action::Step>(
	    env_ptr_, ns + "step", std::bind(&RosAPI::HandleGoal, this, std::placeholders::_1, std::placeholders::_2),
	    std::bind(&RosAPI::HandleCancel, this, std::placeholders::_1),
	    std::bind(&RosAPI::OnStepGoal, this, std::placeholders::_1));
}

void RosAPI::UpdateDynamicParams()
{
	std::vector<rclcpp::Parameter> parameters;
	{
		RecursiveLock lock(env_ptr_->physics_thread_mutex_);
		if (env_ptr_->model_ == nullptr) {
			return;
		}

		const auto &opt = env_ptr_->model_->opt;
		parameters.emplace_back("running", env_ptr_->GetControlSnapshot().running);
		parameters.emplace_back("admin_hash", std::string(env_ptr_->settings_.admin_hash));

		parameters.emplace_back("integrator", opt.integrator);
		parameters.emplace_back("cone", opt.cone);
		parameters.emplace_back("jacobian", opt.jacobian);
		parameters.emplace_back("solver", opt.solver);
		parameters.emplace_back("timestep", opt.timestep);
		parameters.emplace_back("iterations", opt.iterations);
		parameters.emplace_back("tolerance", opt.tolerance);
		parameters.emplace_back("ls_iter", opt.ls_iterations);
		parameters.emplace_back("ls_tol", opt.ls_tolerance);
		parameters.emplace_back("noslip_iter", opt.noslip_iterations);
		parameters.emplace_back("noslip_tol", opt.noslip_tolerance);
		parameters.emplace_back("ccd_iter", opt.ccd_iterations);
		parameters.emplace_back("ccd_tol", opt.ccd_tolerance);
		parameters.emplace_back("sdf_iter", opt.sdf_iterations);
		parameters.emplace_back("sdf_init", opt.sdf_initpoints);

		parameters.emplace_back("gravity", ArrayToString(opt.gravity, 3));
		parameters.emplace_back("wind", ArrayToString(opt.wind, 3));
		parameters.emplace_back("magnetic", ArrayToString(opt.magnetic, 3));
		parameters.emplace_back("density", opt.density);
		parameters.emplace_back("viscosity", opt.viscosity);
		parameters.emplace_back("impratio", opt.impratio);

		parameters.emplace_back("constraint_disabled", static_cast<bool>(opt.disableflags & mjDSBL_CONSTRAINT));
		parameters.emplace_back("equality_disabled", static_cast<bool>(opt.disableflags & mjDSBL_EQUALITY));
		parameters.emplace_back("frictionloss_disabled", static_cast<bool>(opt.disableflags & mjDSBL_FRICTIONLOSS));
		parameters.emplace_back("limit_disabled", static_cast<bool>(opt.disableflags & mjDSBL_LIMIT));
		parameters.emplace_back("contact_disabled", static_cast<bool>(opt.disableflags & mjDSBL_CONTACT));
		parameters.emplace_back("passive_disabled", static_cast<bool>(opt.disableflags & mjDSBL_PASSIVE));
		parameters.emplace_back("gravity_disabled", static_cast<bool>(opt.disableflags & mjDSBL_GRAVITY));
		parameters.emplace_back("clampctrl_disabled", static_cast<bool>(opt.disableflags & mjDSBL_CLAMPCTRL));
		parameters.emplace_back("warmstart_disabled", static_cast<bool>(opt.disableflags & mjDSBL_WARMSTART));
		parameters.emplace_back("filterparent_disabled", static_cast<bool>(opt.disableflags & mjDSBL_FILTERPARENT));
		parameters.emplace_back("actuation_disabled", static_cast<bool>(opt.disableflags & mjDSBL_ACTUATION));
		parameters.emplace_back("refsafe_disabled", static_cast<bool>(opt.disableflags & mjDSBL_REFSAFE));
		parameters.emplace_back("sensor_disabled", static_cast<bool>(opt.disableflags & mjDSBL_SENSOR));
		parameters.emplace_back("midphase_disabled", static_cast<bool>(opt.disableflags & mjDSBL_MIDPHASE));
		parameters.emplace_back("eulerdamp_disabled", static_cast<bool>(opt.disableflags & mjDSBL_EULERDAMP));

		parameters.emplace_back("override_contacts", static_cast<bool>(opt.enableflags & mjENBL_OVERRIDE));
		parameters.emplace_back("energy", static_cast<bool>(opt.enableflags & mjENBL_ENERGY));
		parameters.emplace_back("fwd_inv", static_cast<bool>(opt.enableflags & mjENBL_FWDINV));
		parameters.emplace_back("inv_discrete", static_cast<bool>(opt.enableflags & mjENBL_INVDISCRETE));
		parameters.emplace_back("multiccd", static_cast<bool>(opt.enableflags & mjENBL_MULTICCD));
		parameters.emplace_back("island", static_cast<bool>(opt.enableflags & mjENBL_ISLAND));

		parameters.emplace_back("margin", opt.o_margin);
		parameters.emplace_back("solimp", ArrayToString(opt.o_solimp, mjNIMP));
		parameters.emplace_back("solref", ArrayToString(opt.o_solref, mjNREF));
		parameters.emplace_back("friction", ArrayToString(opt.o_friction, 5));
	}

	ScopedBoolFlag scoped_syncing_dynamic_params(syncing_dynamic_params);
	auto results = env_ptr_->set_parameters(parameters);
	for (const auto &result : results) {
		if (!result.successful) {
			MJR_WARN_STREAM("Failed to sync runtime parameter from model: " << result.reason);
		}
	}
}

rcl_interfaces::msg::SetParametersResult RosAPI::DynamicParamsCallback(const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;

	if (syncing_dynamic_params) {
		return result;
	}

	static const std::unordered_set<std::string> model_backed_parameters = { "integrator",
		                                                                      "cone",
		                                                                      "jacobian",
		                                                                      "solver",
		                                                                      "timestep",
		                                                                      "iterations",
		                                                                      "tolerance",
		                                                                      "ls_iter",
		                                                                      "ls_tol",
		                                                                      "noslip_iter",
		                                                                      "noslip_tol",
		                                                                      "ccd_iter",
		                                                                      "ccd_tol",
		                                                                      "sdf_iter",
		                                                                      "sdf_init",
		                                                                      "gravity",
		                                                                      "wind",
		                                                                      "magnetic",
		                                                                      "density",
		                                                                      "viscosity",
		                                                                      "impratio",
		                                                                      "constraint_disabled",
		                                                                      "equality_disabled",
		                                                                      "frictionloss_disabled",
		                                                                      "limit_disabled",
		                                                                      "contact_disabled",
		                                                                      "passive_disabled",
		                                                                      "gravity_disabled",
		                                                                      "clampctrl_disabled",
		                                                                      "warmstart_disabled",
		                                                                      "filterparent_disabled",
		                                                                      "actuation_disabled",
		                                                                      "refsafe_disabled",
		                                                                      "sensor_disabled",
		                                                                      "midphase_disabled",
		                                                                      "eulerdamp_disabled",
		                                                                      "override_contacts",
		                                                                      "energy",
		                                                                      "fwd_inv",
		                                                                      "inv_discrete",
		                                                                      "multiccd",
		                                                                      "island",
		                                                                      "margin",
		                                                                      "solimp",
		                                                                      "solref",
		                                                                      "friction" };

	bool handles_runtime = false;
	bool touches_model   = false;
	for (const auto &parameter : parameters) {
		const auto &name = parameter.get_name();
		if (name == "running" || name == "admin_hash") {
			handles_runtime = true;
		} else if (model_backed_parameters.find(name) != model_backed_parameters.end()) {
			handles_runtime = true;
			touches_model   = true;
		}
	}

	if (!handles_runtime) {
		// Parameters such as cam_config.* may be declared from the offscreen render thread while model loading
		// waits for render initialization. Avoid taking the physics mutex for parameters this callback does not own.
		return result;
	}

	if (touches_model && env_ptr_->model_ == nullptr) {
		result.successful = false;
		result.reason     = "Cannot update MuJoCo model parameter before a model is loaded.";
		return result;
	}

	RecursiveLock lock(env_ptr_->physics_thread_mutex_);

	for (const auto &parameter : parameters) {
		const auto &name = parameter.get_name();

		try {
			if (name == "running") {
				if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
					throw std::runtime_error("'running' must be a boolean parameter");
				}
				env_ptr_->ApplyPauseState(!parameter.as_bool(), false);
			} else if (name == "admin_hash") {
				if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
					throw std::runtime_error("'admin_hash' must be a string parameter");
				}
				mju::strcpy_arr(env_ptr_->settings_.admin_hash, parameter.as_string().c_str());
			} else if (name == "integrator") {
				ValidateIntegerRange(parameter, 0, 3);
				env_ptr_->model_->opt.integrator = parameter.as_int();
			} else if (name == "cone") {
				ValidateIntegerRange(parameter, 0, 1);
				env_ptr_->model_->opt.cone = parameter.as_int();
			} else if (name == "jacobian") {
				ValidateIntegerRange(parameter, 0, 2);
				env_ptr_->model_->opt.jacobian = parameter.as_int();
			} else if (name == "solver") {
				ValidateIntegerRange(parameter, 0, 2);
				env_ptr_->model_->opt.solver = parameter.as_int();
			} else if (name == "timestep") {
				env_ptr_->model_->opt.timestep = parameter.as_double();
			} else if (name == "iterations") {
				env_ptr_->model_->opt.iterations = parameter.as_int();
			} else if (name == "tolerance") {
				env_ptr_->model_->opt.tolerance = parameter.as_double();
			} else if (name == "ls_iter") {
				env_ptr_->model_->opt.ls_iterations = parameter.as_int();
			} else if (name == "ls_tol") {
				env_ptr_->model_->opt.ls_tolerance = parameter.as_double();
			} else if (name == "noslip_iter") {
				env_ptr_->model_->opt.noslip_iterations = parameter.as_int();
			} else if (name == "noslip_tol") {
				env_ptr_->model_->opt.noslip_tolerance = parameter.as_double();
			} else if (name == "ccd_iter") {
				env_ptr_->model_->opt.ccd_iterations = parameter.as_int();
			} else if (name == "ccd_tol") {
				env_ptr_->model_->opt.ccd_tolerance = parameter.as_double();
			} else if (name == "sdf_iter") {
				env_ptr_->model_->opt.sdf_iterations = parameter.as_int();
			} else if (name == "sdf_init") {
				env_ptr_->model_->opt.sdf_initpoints = parameter.as_int();
			} else if (name == "gravity") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.gravity, 3, result.reason)) {
					result.successful = false;
					return result;
				}
			} else if (name == "wind") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.wind, 3, result.reason)) {
					result.successful = false;
					return result;
				}
			} else if (name == "magnetic") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.magnetic, 3, result.reason)) {
					result.successful = false;
					return result;
				}
			} else if (name == "density") {
				env_ptr_->model_->opt.density = parameter.as_double();
			} else if (name == "viscosity") {
				env_ptr_->model_->opt.viscosity = parameter.as_double();
			} else if (name == "impratio") {
				env_ptr_->model_->opt.impratio = parameter.as_double();
			} else if (name == "constraint_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 0, parameter.as_bool());
			} else if (name == "equality_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 1, parameter.as_bool());
			} else if (name == "frictionloss_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 2, parameter.as_bool());
			} else if (name == "limit_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 3, parameter.as_bool());
			} else if (name == "contact_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 4, parameter.as_bool());
			} else if (name == "passive_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 5, parameter.as_bool());
			} else if (name == "gravity_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 6, parameter.as_bool());
			} else if (name == "clampctrl_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 7, parameter.as_bool());
			} else if (name == "warmstart_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 8, parameter.as_bool());
			} else if (name == "filterparent_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 9, parameter.as_bool());
			} else if (name == "actuation_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 10, parameter.as_bool());
			} else if (name == "refsafe_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 11, parameter.as_bool());
			} else if (name == "sensor_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 12, parameter.as_bool());
			} else if (name == "midphase_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 13, parameter.as_bool());
			} else if (name == "eulerdamp_disabled") {
				util::bit_set_to(env_ptr_->model_->opt.disableflags, 14, parameter.as_bool());
			} else if (name == "override_contacts") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 0, parameter.as_bool());
			} else if (name == "energy") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 1, parameter.as_bool());
			} else if (name == "fwd_inv") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 2, parameter.as_bool());
			} else if (name == "inv_discrete") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 3, parameter.as_bool());
			} else if (name == "multiccd") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 4, parameter.as_bool());
			} else if (name == "island") {
				util::bit_set_to(env_ptr_->model_->opt.enableflags, 5, parameter.as_bool());
			} else if (name == "margin") {
				env_ptr_->model_->opt.o_margin = parameter.as_double();
			} else if (name == "solimp") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.o_solimp, mjNIMP, result.reason)) {
					result.successful = false;
					return result;
				}
			} else if (name == "solref") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.o_solref, mjNREF, result.reason)) {
					result.successful = false;
					return result;
				}
			} else if (name == "friction") {
				if (!ParseMjtNumArray(parameter.as_string(), env_ptr_->model_->opt.o_friction, 5, result.reason)) {
					result.successful = false;
					return result;
				}
			}
		} catch (const std::exception &e) {
			result.successful = false;
			result.reason     = "Failed to set parameter '" + name + "': " + e.what();
			return result;
		}
	}

	return result;
}

void RosAPI::OnStepGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>> &goal_handle)
{
	std::thread{ std::bind(&RosAPI::ExecuteStepGoal, this, std::placeholders::_1), goal_handle }.detach();
}

void RosAPI::ExecuteStepGoal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<mujoco_ros_msgs::action::Step>> &goal_handle)
{
	const auto goal  = goal_handle->get_goal();
	auto feedback    = std::make_shared<mujoco_ros_msgs::action::Step::Feedback>();
	auto &steps_left = feedback->steps_left;
	auto result      = std::make_shared<mujoco_ros_msgs::action::Step::Result>();

	if (!env_ptr_->RequestManualSteps(goal->num_steps)) {
		MJR_WARN("Simulation is currently unpaused. Stepping makes no sense right now.");
		result->success = false;
		goal_handle->abort(result);
		MJR_DEBUG("Aborted step goal");
		return;
	}

	steps_left = goal->num_steps;

	result->success = true;
	while (env_ptr_->GetControlSnapshot().pending_steps > 0) {
		const auto control_snapshot = env_ptr_->GetControlSnapshot();
		if (goal_handle->is_canceling() || !rclcpp::ok() || control_snapshot.shutdown_requested ||
		    control_snapshot.load_request > 0 || control_snapshot.reset_requested) {
			MJR_WARN("Simulation step action preempted");
			steps_left = util::as_unsigned(control_snapshot.pending_steps);
			goal_handle->publish_feedback(feedback);
			result->success = false;
			goal_handle->canceled(result);
			env_ptr_->CancelManualSteps();
			return;
		}

		steps_left = util::as_unsigned(control_snapshot.pending_steps);
		goal_handle->publish_feedback(feedback);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	steps_left = util::as_unsigned(env_ptr_->GetControlSnapshot().pending_steps);
	goal_handle->publish_feedback(feedback);
	goal_handle->succeed(result);
}

void RosAPI::SetPauseCB(const mujoco_ros_msgs::srv::SetPause::Request::SharedPtr &req,
                        const mujoco_ros_msgs::srv::SetPause::Response::SharedPtr &res)
{
	res->success = env_ptr_->TogglePaused(req->paused, req->admin_hash);
}

void RosAPI::ShutdownCB(const std_srvs::srv::Empty::Request::SharedPtr & /*req*/,
                        const std_srvs::srv::Empty::Response::SharedPtr & /*res*/)
{
	env_ptr_->Shutdown();
}

void RosAPI::ReloadCB(const mujoco_ros_msgs::srv::Reload::Request::SharedPtr &req,
                      const mujoco_ros_msgs::srv::Reload::Response::SharedPtr &res)
{
	char load_error[MujocoEnv::kErrorLength];
	res->success        = env_ptr_->LoadModelFromString(req->model, load_error);
	res->status_message = load_error;
}

void RosAPI::ResetCB(const std_srvs::srv::Empty::Request::SharedPtr & /*req*/,
                     const std_srvs::srv::Empty::Response::SharedPtr & /*res*/)
{
	env_ptr_->Reset();
}

void RosAPI::SetBodyStateCB(const mujoco_ros_msgs::srv::SetBodyState::Request::SharedPtr &req,
                            const mujoco_ros_msgs::srv::SetBodyState::Response::SharedPtr &res)
{
	geometry_msgs::msg::PoseStamped target_pose;

	mjtNum state[15] = { 0 };
	mjtNum *pose     = state;
	mjtNum *quat     = pose + 3;
	mjtNum *twist    = pose + 7;
	mjtNum *mass     = twist + 6;

	std::string full_error_msg;

	bool resp_failure_override = false;

	if (req->set_pose) {
		geometry_msgs::msg::PoseStamped init_pose = req->state.pose;
		bool valid_pose                           = true;
		if (!init_pose.header.frame_id.empty() && init_pose.header.frame_id != "world") {
			try {
				env_ptr_->tf_buffer_->transform<geometry_msgs::msg::PoseStamped>(init_pose, target_pose, "world");
			} catch (tf2::TransformException &ex) {
				MJR_WARN_STREAM(ex.what());
				full_error_msg +=
				    "Could not transform frame '" + req->state.pose.header.frame_id + "' to frame world" + '\n';
				resp_failure_override = true;
				valid_pose            = false;
			}
		} else {
			target_pose = req->state.pose;
		}

		if (valid_pose) {
			pose[0] = target_pose.pose.position.x;
			pose[1] = target_pose.pose.position.y;
			pose[2] = target_pose.pose.position.z;
			pose[3] = target_pose.pose.orientation.w;
			pose[4] = target_pose.pose.orientation.x;
			pose[5] = target_pose.pose.orientation.y;
			pose[6] = target_pose.pose.orientation.z;

			mju_normalize4(quat);
		} else {
			MJR_WARN("Invalid pose, not setting pose");
			req->set_pose = false;
		}
	}

	if (req->set_twist) {
		// Only pose can be transformed. Twist will be ignored!
		if (!req->state.twist.header.frame_id.empty() && req->state.twist.header.frame_id != "world") {
			std::string error_msg("Transforming twists from other frames is not supported! Not setting twist.");
			MJR_WARN_STREAM(error_msg);
			full_error_msg += error_msg + '\n';
			resp_failure_override = true;
		} else {
			twist[0] = req->state.twist.twist.linear.x;
			twist[1] = req->state.twist.twist.linear.y;
			twist[2] = req->state.twist.twist.linear.z;
			twist[3] = req->state.twist.twist.angular.x;
			twist[4] = req->state.twist.twist.angular.y;
			twist[5] = req->state.twist.twist.angular.z;
		}
	}

	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	*mass                                    = static_cast<mjtNum>(req->state.mass);
	res->success =
	    env_ptr_->SetBodyState(req->state.name, pose, twist, *mass, req->set_pose, req->set_twist, req->set_mass,
	                           req->reset_qpos, req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	if (resp_failure_override) {
		res->success        = false;
		res->status_message = full_error_msg + "\n" + std::string(status_msg);
	}
}

void RosAPI::GetBodyStateCB(const mujoco_ros_msgs::srv::GetBodyState::Request::SharedPtr &req,
                            const mujoco_ros_msgs::srv::GetBodyState::Response::SharedPtr &res)
{
	std::string body_name = req->name;
	mjtNum state[14]      = { 0 };
	mjtNum *pose          = state;
	mjtNum *twist         = pose + 7;
	mjtNum *mass          = twist + 6;

	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success =
	    env_ptr_->GetBodyState(body_name, pose, twist, mass, req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	res->status_message = std::string(status_msg);

	res->state.name = body_name;
	res->state.mass = static_cast<decltype(res->state.mass)>(*mass);

	res->state.pose.header             = std_msgs::msg::Header();
	res->state.pose.header.frame_id    = "world";
	res->state.pose.pose.position.x    = pose[0];
	res->state.pose.pose.position.y    = pose[1];
	res->state.pose.pose.position.z    = pose[2];
	res->state.pose.pose.orientation.w = pose[3];
	res->state.pose.pose.orientation.x = pose[4];
	res->state.pose.pose.orientation.y = pose[5];
	res->state.pose.pose.orientation.z = pose[6];

	res->state.twist.header          = std_msgs::msg::Header();
	res->state.twist.header.frame_id = "world";
	res->state.twist.twist.linear.x  = twist[0];
	res->state.twist.twist.linear.y  = twist[1];
	res->state.twist.twist.linear.z  = twist[2];
	res->state.twist.twist.angular.x = twist[3];
	res->state.twist.twist.angular.y = twist[4];
	res->state.twist.twist.angular.z = twist[5];
}

void RosAPI::SetGravityCB(const mujoco_ros_msgs::srv::SetGravity::Request::SharedPtr &req,
                          const mujoco_ros_msgs::srv::SetGravity::Response::SharedPtr &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res->success = env_ptr_->SetGravity(req->gravity.data(), req->admin_hash, status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);
}

void RosAPI::GetGravityCB(const mujoco_ros_msgs::srv::GetGravity::Request::SharedPtr &req,
                          const mujoco_ros_msgs::srv::GetGravity::Response::SharedPtr &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success = env_ptr_->GetGravity(res->gravity.data(), req->admin_hash, status_msg, MujocoEnv::kErrorLength);

	res->status_message = std::string(status_msg);
}

void RosAPI::SetGeomPropertiesCB(const mujoco_ros_msgs::srv::SetGeomProperties::Request::SharedPtr &req,
                                 const mujoco_ros_msgs::srv::SetGeomProperties::Response::SharedPtr &res)
{
	char status_msg[MujocoEnv::kErrorLength] = { 0 };
	res->success                             = env_ptr_->SetGeomProperties(
       req->properties.name, req->properties.body_mass, req->properties.friction_slide, req->properties.friction_spin,
       req->properties.friction_roll, req->properties.size_0, req->properties.size_1, req->properties.size_2,
       req->properties.type.value, req->set_mass, req->set_friction, req->set_type, req->set_size, req->admin_hash,
       status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);
}

void RosAPI::GetGeomPropertiesCB(const mujoco_ros_msgs::srv::GetGeomProperties::Request::SharedPtr &req,
                                 const mujoco_ros_msgs::srv::GetGeomProperties::Response::SharedPtr &res)
{
	mjtNum properties[8]                     = { 0 };
	char status_msg[MujocoEnv::kErrorLength] = { 0 };

	res->success        = env_ptr_->GetGeomProperties(req->geom_name,
	                                                  properties[0], // body_mass
	                                                  properties[1], // friction_slide
	                                                  properties[2], // friction_spin
	                                                  properties[3], // friction_roll
	                                                  properties[4], // size_x
	                                                  properties[5], // size_y
	                                                  properties[6], // size_z
	                                                  properties[7], // type
	                                                  req->admin_hash, status_msg, MujocoEnv::kErrorLength);
	res->status_message = std::string(status_msg);

	res->properties.name           = req->geom_name;
	res->properties.body_mass      = static_cast<decltype(res->properties.body_mass)>(properties[0]);
	res->properties.friction_slide = static_cast<decltype(res->properties.friction_slide)>(properties[1]);
	res->properties.friction_spin  = static_cast<decltype(res->properties.friction_spin)>(properties[2]);
	res->properties.friction_roll  = static_cast<decltype(res->properties.friction_roll)>(properties[3]);
	res->properties.size_0         = static_cast<decltype(res->properties.size_0)>(properties[4]);
	res->properties.size_1         = static_cast<decltype(res->properties.size_1)>(properties[5]);
	res->properties.size_2         = static_cast<decltype(res->properties.size_2)>(properties[6]);
	res->properties.type.value     = static_cast<decltype(res->properties.type.value)>(properties[7]);
}

bool RosAPI::SetEqualityConstraintParameters(const mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
                                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	// TODO: add set_bools to message to not require all parameters to be set
	// 3 for anchor, 7 for relpose, mjNEQDATA for polycoef, 1 for torquescale, mjNIMP + mjNREF for solimp and solref
	// (solver_params)
	mjtNum params[3 + 7 + mjNEQDATA + 1 + mjNIMP + mjNREF] = { 0 };
	mjtNum *anchor                                         = params;
	mjtNum *relpose                                        = anchor + 3;
	mjtNum *polycoef                                       = relpose + 7;
	mjtNum *torquescale                                    = polycoef + mjNEQDATA;
	mjtNum *solver_params                                  = torquescale + 1;

	anchor[0] = parameters.anchor.x;
	anchor[1] = parameters.anchor.y;
	anchor[2] = parameters.anchor.z;

	relpose[0] = parameters.relpose.position.x;
	relpose[1] = parameters.relpose.position.y;
	relpose[2] = parameters.relpose.position.z;
	relpose[3] = parameters.relpose.orientation.w;
	relpose[4] = parameters.relpose.orientation.x;
	relpose[5] = parameters.relpose.orientation.y;
	relpose[6] = parameters.relpose.orientation.z;

	mju_copy(polycoef, parameters.polycoef.data(), parameters.polycoef.size());
	torquescale[0] = parameters.torquescale;

	solver_params[0] = parameters.solver_parameters.dmin;
	solver_params[1] = parameters.solver_parameters.dmax;
	solver_params[2] = parameters.solver_parameters.width;
	solver_params[3] = parameters.solver_parameters.midpoint;
	solver_params[4] = parameters.solver_parameters.power;
	solver_params[5] = parameters.solver_parameters.timeconst;
	solver_params[6] = parameters.solver_parameters.dampratio;

	return env_ptr_->SetEqualityConstraintParameters(
	    parameters.name, parameters.type.value, solver_params, parameters.active, parameters.element1,
	    parameters.element2, *torquescale, anchor, relpose, polycoef, admin_hash, status_message, status_sz);
}

void RosAPI::SetEqualityConstraintParametersArrayCB(
    const mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Request::SharedPtr &req,
    const mujoco_ros_msgs::srv::SetEqualityConstraintParameters::Response::SharedPtr &res)
{
	res->success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;

	for (const auto &parameters : req->parameters) {
		error_msg[0]  = '\0';
		bool success  = SetEqualityConstraintParameters(parameters, req->admin_hash, error_msg, MujocoEnv::kErrorLength);
		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (!success) {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Not all constraints could be set";
		res->status_message = status_message;
		res->success        = false;
	} else if (failed_any) {
		status_message += "Could not set any constraints";
		res->status_message = status_message;
		res->success        = false;
	} else if (!failed_any && !succeeded_any) {
		status_message += "No constraints provided in request";
		res->status_message = status_message;
		res->success        = false;
	}
}

bool RosAPI::GetEqualityConstraintParameters(mujoco_ros_msgs::msg::EqualityConstraintParameters &parameters,
                                             const std::string &admin_hash, char *status_message, const int status_sz)
{
	// 3 for anchor, 7 for relpose, mjNEQDATA for polycoef, 1 for torquescale, mjNIMP + mjNREF for solimp and solref
	// (solver_params)
	mjtNum params[3 + 7 + mjNEQDATA + 1 + mjNIMP + mjNREF] = { 0 };
	mjtNum *anchor                                         = params;
	mjtNum *relpose                                        = anchor + 3;
	mjtNum *polycoef                                       = relpose + 7;
	mjtNum *torquescale                                    = polycoef + mjNEQDATA;
	mjtNum *solver_params                                  = torquescale + 1;
	int type;
	bool active, success;

	success = env_ptr_->GetEqualityConstraintParameters(parameters.name, type, solver_params, active,
	                                                    parameters.element1, parameters.element2, *torquescale, anchor,
	                                                    relpose, polycoef, admin_hash, status_message, status_sz);

	parameters.type.value = static_cast<decltype(parameters.type.value)>(type);
	parameters.active     = active;

	parameters.anchor.x = anchor[0];
	parameters.anchor.y = anchor[1];
	parameters.anchor.z = anchor[2];

	parameters.relpose.position.x    = relpose[0];
	parameters.relpose.position.y    = relpose[1];
	parameters.relpose.position.z    = relpose[2];
	parameters.relpose.orientation.w = relpose[3];
	parameters.relpose.orientation.x = relpose[4];
	parameters.relpose.orientation.y = relpose[5];
	parameters.relpose.orientation.z = relpose[6];

	parameters.polycoef.resize(mjNEQDATA);
	mju_copy(parameters.polycoef.data(), polycoef, mjNEQDATA);

	parameters.torquescale = torquescale[0];

	parameters.solver_parameters.dmin      = solver_params[0];
	parameters.solver_parameters.dmax      = solver_params[1];
	parameters.solver_parameters.width     = solver_params[2];
	parameters.solver_parameters.midpoint  = solver_params[3];
	parameters.solver_parameters.power     = solver_params[4];
	parameters.solver_parameters.timeconst = solver_params[5];
	parameters.solver_parameters.dampratio = solver_params[6];

	return success;
}

void RosAPI::GetEqualityConstraintParametersArrayCB(
    const mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Request::SharedPtr &req,
    const mujoco_ros_msgs::srv::GetEqualityConstraintParameters::Response::SharedPtr &res)
{
	res->success = true;

	bool failed_any    = false;
	bool succeeded_any = false;
	char error_msg[MujocoEnv::kErrorLength];
	std::string status_message;
	for (const auto &name : req->names) {
		error_msg[0] = '\0';
		mujoco_ros_msgs::msg::EqualityConstraintParameters eqc;
		eqc.name     = name;
		bool success = GetEqualityConstraintParameters(eqc, req->admin_hash, error_msg, MujocoEnv::kErrorLength);

		failed_any    = (failed_any || !success);
		succeeded_any = (succeeded_any || success);
		if (success) {
			res->parameters.emplace_back(eqc);
		} else {
			status_message += std::string(error_msg) + '\n';
		}
	}

	if (succeeded_any && failed_any) {
		status_message += "Result: Not all constraints could be fetched";
		res->status_message = status_message;
		res->success        = false;
	} else if (failed_any) {
		status_message += "Result: Could not fetch any constraints";
		res->status_message = status_message;
		res->success        = false;
	}
}

void RosAPI::GetStateUintCB(const mujoco_ros_msgs::srv::GetStateUint::Request::SharedPtr & /*req*/,
                            const mujoco_ros_msgs::srv::GetStateUint::Response::SharedPtr &res)
{
	int status;
	env_ptr_->GetSimulationStatus(status, res->state.description);
	res->state.value = static_cast<decltype(res->state.value)>(status);
}

void RosAPI::GetSimInfoCB(const mujoco_ros_msgs::srv::GetSimInfo::Request::SharedPtr & /*req*/,
                          const mujoco_ros_msgs::srv::GetSimInfo::Response::SharedPtr &res)
{
	bool valid, paused;
	int load_count, loading_state, pending_sim_steps;
	env_ptr_->GetSimInfo(res->state.model_path, valid, load_count, loading_state, res->state.loading_state.description,
	                     paused, pending_sim_steps, res->state.rt_measured, res->state.rt_setting);
	res->state.model_valid         = valid;
	res->state.load_count          = static_cast<decltype(res->state.load_count)>(load_count);
	res->state.loading_state.value = static_cast<decltype(res->state.loading_state.value)>(loading_state);
	res->state.paused              = paused;
	res->state.pending_sim_steps   = static_cast<decltype(res->state.pending_sim_steps)>(pending_sim_steps);
}

void RosAPI::SetRTFactorCB(const mujoco_ros_msgs::srv::SetFloat::Request::SharedPtr &req,
                           const mujoco_ros_msgs::srv::SetFloat::Response::SharedPtr &res)
{
	res->success = env_ptr_->SetRealTimeFactor(static_cast<float>(req->value), req->admin_hash);
}

void RosAPI::GetPluginStatsCB(const mujoco_ros_msgs::srv::GetPluginStats::Request::SharedPtr & /*req*/,
                              const mujoco_ros_msgs::srv::GetPluginStats::Response::SharedPtr &res)
{
	for (const auto &plugin_stat : env_ptr_->GetPluginStats()) {
		mujoco_ros_msgs::msg::PluginStats stats;
		// stats.plugin_name             = plugin_stat.name; // TODO: add plugin name to message
		stats.plugin_type             = plugin_stat.type;
		stats.load_time               = plugin_stat.load_time;
		stats.reset_time              = plugin_stat.reset_time;
		stats.ema_steptime_control    = plugin_stat.ema_steptime_control;
		stats.ema_steptime_passive    = plugin_stat.ema_steptime_passive;
		stats.ema_steptime_render     = plugin_stat.ema_steptime_render;
		stats.ema_steptime_last_stage = plugin_stat.ema_steptime_last_stage;
		res->stats.emplace_back(stats);
	}
}

void RosAPI::LoadInitialJointStatesCB(const std_srvs::srv::Empty::Request::SharedPtr & /*req*/,
                                      const std_srvs::srv::Empty::Response::SharedPtr & /*res*/)
{
	env_ptr_->LoadInitialJointStates();
}

void RosAPI::SetupClockPublisher()
{
	if (env_ptr_->settings_.use_sim_time) {
		MJR_DEBUG("Setting up clock publisher");
		rclcpp::QoS clock_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
		clock_pub_            = env_ptr_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", clock_qos);
		PublishSimTime(mjtNum(0));
	}
}

void RosAPI::PublishSimTime(mjtNum time)
{
	if (!env_ptr_->settings_.use_sim_time) {
		return;
	}
	rosgraph_msgs::msg::Clock::UniquePtr ros_time = std::make_unique<rosgraph_msgs::msg::Clock>();
	ros_time->clock = rclcpp::Time(static_cast<uint64_t>(time * 1e9)); // convert to nanoseconds
	clock_pub_->publish(std::move(ros_time));
}

} // namespace mujoco_ros
