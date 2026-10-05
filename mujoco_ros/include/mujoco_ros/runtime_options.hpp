#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

#include <mujoco_ros/generation.hpp>

namespace mujoco_ros {

using RuntimeOptionValue = std::variant<bool, std::int64_t, double, std::string>;

struct RuntimeOptionInput
{
	std::string field;
	RuntimeOptionValue value;
};

struct RuntimeOptionsSnapshot
{
	int integrator = mjINT_EULER;
	int cone       = mjCONE_ELLIPTIC;
	int jacobian   = mjJAC_AUTO;
	int solver     = mjSOL_NEWTON;

	double timestep         = 1e-3;
	int iterations          = 100;
	double tolerance        = 1e-8;
	int ls_iterations       = 50;
	double ls_tolerance     = 0.01;
	int noslip_iterations   = 0;
	double noslip_tolerance = 1e-6;
	int ccd_iterations      = 50;
	double ccd_tolerance    = 1e-6;
	int sdf_iterations      = 10;
	int sdf_initpoints      = 40;
	double density          = 0.0;
	double viscosity        = 0.0;
	double impratio         = 1.0;
	double margin           = 0.0;

	std::array<double, 3> gravity  = { 0.0, 0.0, -9.81 };
	std::array<double, 3> wind     = { 0.0, 0.0, 0.0 };
	std::array<double, 3> magnetic = { 0.0, -0.5, 0.0 };
	std::array<double, 5> solimp   = { 0.9, 0.95, 0.001, 0.5, 2.0 };
	std::array<double, 2> solref   = { 0.02, 1.0 };
	std::array<double, 5> friction = { 1.0, 1.0, 0.005, 0.0001, 0.0001 };

	bool constraint_disabled   = false;
	bool equality_disabled     = false;
	bool frictionloss_disabled = false;
	bool limit_disabled        = false;
	bool contact_disabled      = false;
	bool passive_disabled      = false;
	bool gravity_disabled      = false;
	bool clampctrl_disabled    = false;
	bool warmstart_disabled    = false;
	bool filterparent_disabled = false;
	bool actuation_disabled    = false;
	bool refsafe_disabled      = false;
	bool sensor_disabled       = false;
	bool midphase_disabled     = false;
	bool eulerdamp_disabled    = false;

	bool override_contacts = false;
	bool energy            = false;
	bool fwd_inv           = false;
	bool inv_discrete      = false;
	bool multiccd          = false;
	bool island            = false;

	friend bool operator==(const RuntimeOptionsSnapshot &lhs, const RuntimeOptionsSnapshot &rhs)
	{
		return std::tie(lhs.integrator, lhs.cone, lhs.jacobian, lhs.solver, lhs.timestep, lhs.iterations, lhs.tolerance,
		                lhs.ls_iterations, lhs.ls_tolerance, lhs.noslip_iterations, lhs.noslip_tolerance,
		                lhs.ccd_iterations, lhs.ccd_tolerance, lhs.sdf_iterations, lhs.sdf_initpoints, lhs.density,
		                lhs.viscosity, lhs.impratio, lhs.margin, lhs.gravity, lhs.wind, lhs.magnetic, lhs.solimp,
		                lhs.solref, lhs.friction, lhs.constraint_disabled, lhs.equality_disabled,
		                lhs.frictionloss_disabled, lhs.limit_disabled, lhs.contact_disabled, lhs.passive_disabled,
		                lhs.gravity_disabled, lhs.clampctrl_disabled, lhs.warmstart_disabled, lhs.filterparent_disabled,
		                lhs.actuation_disabled, lhs.refsafe_disabled, lhs.sensor_disabled, lhs.midphase_disabled,
		                lhs.eulerdamp_disabled, lhs.override_contacts, lhs.energy, lhs.fwd_inv, lhs.inv_discrete,
		                lhs.multiccd, lhs.island) ==
		       std::tie(rhs.integrator, rhs.cone, rhs.jacobian, rhs.solver, rhs.timestep, rhs.iterations, rhs.tolerance,
		                rhs.ls_iterations, rhs.ls_tolerance, rhs.noslip_iterations, rhs.noslip_tolerance,
		                rhs.ccd_iterations, rhs.ccd_tolerance, rhs.sdf_iterations, rhs.sdf_initpoints, rhs.density,
		                rhs.viscosity, rhs.impratio, rhs.margin, rhs.gravity, rhs.wind, rhs.magnetic, rhs.solimp,
		                rhs.solref, rhs.friction, rhs.constraint_disabled, rhs.equality_disabled,
		                rhs.frictionloss_disabled, rhs.limit_disabled, rhs.contact_disabled, rhs.passive_disabled,
		                rhs.gravity_disabled, rhs.clampctrl_disabled, rhs.warmstart_disabled, rhs.filterparent_disabled,
		                rhs.actuation_disabled, rhs.refsafe_disabled, rhs.sensor_disabled, rhs.midphase_disabled,
		                rhs.eulerdamp_disabled, rhs.override_contacts, rhs.energy, rhs.fwd_inv, rhs.inv_discrete,
		                rhs.multiccd, rhs.island);
	}
};

struct RuntimeOptionsPatch
{
	std::optional<int> integrator;
	std::optional<int> cone;
	std::optional<int> jacobian;
	std::optional<int> solver;
	std::optional<double> timestep;
	std::optional<int> iterations;
	std::optional<double> tolerance;
	std::optional<int> ls_iterations;
	std::optional<double> ls_tolerance;
	std::optional<int> noslip_iterations;
	std::optional<double> noslip_tolerance;
	std::optional<int> ccd_iterations;
	std::optional<double> ccd_tolerance;
	std::optional<int> sdf_iterations;
	std::optional<int> sdf_initpoints;
	std::optional<double> density;
	std::optional<double> viscosity;
	std::optional<double> impratio;
	std::optional<double> margin;
	std::optional<std::array<double, 3>> gravity;
	std::optional<std::array<double, 3>> wind;
	std::optional<std::array<double, 3>> magnetic;
	std::optional<std::array<double, 5>> solimp;
	std::optional<std::array<double, 2>> solref;
	std::optional<std::array<double, 5>> friction;

	std::optional<bool> constraint_disabled;
	std::optional<bool> equality_disabled;
	std::optional<bool> frictionloss_disabled;
	std::optional<bool> limit_disabled;
	std::optional<bool> contact_disabled;
	std::optional<bool> passive_disabled;
	std::optional<bool> gravity_disabled;
	std::optional<bool> clampctrl_disabled;
	std::optional<bool> warmstart_disabled;
	std::optional<bool> filterparent_disabled;
	std::optional<bool> actuation_disabled;
	std::optional<bool> refsafe_disabled;
	std::optional<bool> sensor_disabled;
	std::optional<bool> midphase_disabled;
	std::optional<bool> eulerdamp_disabled;

	std::optional<bool> override_contacts;
	std::optional<bool> energy;
	std::optional<bool> fwd_inv;
	std::optional<bool> inv_discrete;
	std::optional<bool> multiccd;
	std::optional<bool> island;
};

struct RuntimeOptionsError
{
	std::string field;
	std::string message;
};

struct RuntimeOptionsPatchResult
{
	std::optional<RuntimeOptionsPatch> patch;
	std::optional<RuntimeOptionsError> error;
	bool ok() const { return patch.has_value() && !error.has_value(); }
};

struct RuntimeOptionsValidationResult
{
	std::optional<RuntimeOptionsError> error;
	bool ok() const { return !error.has_value(); }
};

struct RuntimeOptionsTransactionResult
{
	std::optional<RuntimeOptionsSnapshot> effective;
	std::optional<RuntimeOptionsError> error;
	OptionsEpoch epoch;
	bool ok() const { return effective.has_value() && !error.has_value(); }
	static RuntimeOptionsTransactionResult Applied(RuntimeOptionsSnapshot snapshot, OptionsEpoch epoch);
	static RuntimeOptionsTransactionResult Rejected(RuntimeOptionsError error, OptionsEpoch epoch);
};

RuntimeOptionsSnapshot ReadRuntimeOptions(const mjOption &option);
RuntimeOptionsPatchResult ParseRuntimeOptionsPatch(const std::vector<RuntimeOptionInput> &input);
RuntimeOptionsValidationResult ValidateRuntimeOptions(const RuntimeOptionsSnapshot &snapshot);
RuntimeOptionsSnapshot MergeRuntimeOptions(const RuntimeOptionsSnapshot &base, const RuntimeOptionsPatch &patch);
void WriteRuntimeOptions(const RuntimeOptionsSnapshot &snapshot, mjOption &option);

} // namespace mujoco_ros
