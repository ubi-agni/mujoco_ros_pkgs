#include <mujoco_ros/runtime_options.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace mujoco_ros {
namespace {

RuntimeOptionsPatchResult RejectPatch(std::string field, std::string message)
{
	return { std::nullopt, RuntimeOptionsError{ std::move(field), std::move(message) } };
}

RuntimeOptionsValidationResult RejectValidation(std::string field, std::string message)
{
	return { RuntimeOptionsError{ std::move(field), std::move(message) } };
}

template <typename T>
bool ReadValue(const RuntimeOptionInput &input, T &value)
{
	if (const auto *typed = std::get_if<T>(&input.value)) {
		value = *typed;
		return true;
	}
	return false;
}

bool ReadInteger(const RuntimeOptionInput &input, int &value)
{
	std::int64_t parsed = 0;
	if (!ReadValue(input, parsed) || parsed < std::numeric_limits<int>::min() ||
	    parsed > std::numeric_limits<int>::max()) {
		return false;
	}
	value = static_cast<int>(parsed);
	return true;
}

template <std::size_t N>
bool ParseArray(const std::string &value, std::array<double, N> &result, std::string &reason)
{
	std::istringstream stream(value);
	std::string token;
	std::size_t count = 0;
	while (stream >> token) {
		if (count == N) {
			reason = "expected exactly " + std::to_string(N) + " values";
			return false;
		}
		try {
			size_t consumed     = 0;
			const double parsed = std::stod(token, &consumed);
			if (consumed != token.size() || !std::isfinite(parsed)) {
				reason = "invalid numeric token '" + token + "'";
				return false;
			}
			result[count++] = parsed;
		} catch (const std::exception &) {
			reason = "invalid numeric token '" + token + "'";
			return false;
		}
	}
	if (count != N) {
		reason = "expected exactly " + std::to_string(N) + " values";
		return false;
	}
	return true;
}

template <std::size_t N>
void ReadArray(const mjtNum (&source)[N], std::array<double, N> &destination)
{
	for (std::size_t i = 0; i < N; ++i) {
		destination[i] = static_cast<double>(source[i]);
	}
}

template <typename T, std::size_t N>
void WriteArray(const std::array<double, N> &source, T (&destination)[N])
{
	for (std::size_t i = 0; i < N; ++i) {
		destination[i] = static_cast<T>(source[i]);
	}
}

template <typename T>
void ApplyOptional(const std::optional<T> &value, T &destination)
{
	if (value) {
		destination = *value;
	}
}

template <typename T, std::size_t N>
void ApplyOptional(const std::optional<std::array<T, N>> &value, std::array<T, N> &destination)
{
	if (value) {
		destination = *value;
	}
}

template <typename T>
bool Finite(T value)
{
	return std::isfinite(static_cast<double>(value));
}

template <std::size_t N>
RuntimeOptionsValidationResult ValidateFiniteArray(const char *field, const std::array<double, N> &values)
{
	for (const auto value : values) {
		if (!Finite(value)) {
			return RejectValidation(field, "all values must be finite");
		}
	}
	return {};
}

template <typename Bits, typename T>
void SetBit(Bits &bits, T bit, bool enabled)
{
	if (enabled) {
		bits |= static_cast<Bits>(bit);
	} else {
		bits &= ~static_cast<Bits>(bit);
	}
}

} // namespace

RuntimeOptionsTransactionResult RuntimeOptionsTransactionResult::Applied(RuntimeOptionsSnapshot snapshot,
                                                                         OptionsEpoch epoch)
{
	return { std::move(snapshot), std::nullopt, epoch };
}

RuntimeOptionsTransactionResult RuntimeOptionsTransactionResult::Rejected(RuntimeOptionsError error, OptionsEpoch epoch)
{
	return { std::nullopt, std::move(error), epoch };
}

RuntimeOptionsSnapshot ReadRuntimeOptions(const mjOption &option)
{
	RuntimeOptionsSnapshot result;
	result.integrator        = option.integrator;
	result.cone              = option.cone;
	result.jacobian          = option.jacobian;
	result.solver            = option.solver;
	result.timestep          = option.timestep;
	result.iterations        = option.iterations;
	result.tolerance         = option.tolerance;
	result.ls_iterations     = option.ls_iterations;
	result.ls_tolerance      = option.ls_tolerance;
	result.noslip_iterations = option.noslip_iterations;
	result.noslip_tolerance  = option.noslip_tolerance;
	result.ccd_iterations    = option.ccd_iterations;
	result.ccd_tolerance     = option.ccd_tolerance;
	result.sdf_iterations    = option.sdf_iterations;
	result.sdf_initpoints    = option.sdf_initpoints;
	result.density           = option.density;
	result.viscosity         = option.viscosity;
	result.impratio          = option.impratio;
	result.margin            = option.o_margin;
	ReadArray(option.gravity, result.gravity);
	ReadArray(option.wind, result.wind);
	ReadArray(option.magnetic, result.magnetic);
	ReadArray(option.o_solimp, result.solimp);
	ReadArray(option.o_solref, result.solref);
	ReadArray(option.o_friction, result.friction);

	result.constraint_disabled   = (option.disableflags & mjDSBL_CONSTRAINT) != 0;
	result.equality_disabled     = (option.disableflags & mjDSBL_EQUALITY) != 0;
	result.frictionloss_disabled = (option.disableflags & mjDSBL_FRICTIONLOSS) != 0;
	result.limit_disabled        = (option.disableflags & mjDSBL_LIMIT) != 0;
	result.contact_disabled      = (option.disableflags & mjDSBL_CONTACT) != 0;
	result.passive_disabled      = (option.disableflags & mjDSBL_PASSIVE) != 0;
	result.gravity_disabled      = (option.disableflags & mjDSBL_GRAVITY) != 0;
	result.clampctrl_disabled    = (option.disableflags & mjDSBL_CLAMPCTRL) != 0;
	result.warmstart_disabled    = (option.disableflags & mjDSBL_WARMSTART) != 0;
	result.filterparent_disabled = (option.disableflags & mjDSBL_FILTERPARENT) != 0;
	result.actuation_disabled    = (option.disableflags & mjDSBL_ACTUATION) != 0;
	result.refsafe_disabled      = (option.disableflags & mjDSBL_REFSAFE) != 0;
	result.sensor_disabled       = (option.disableflags & mjDSBL_SENSOR) != 0;
	result.midphase_disabled     = (option.disableflags & mjDSBL_MIDPHASE) != 0;
	result.eulerdamp_disabled    = (option.disableflags & mjDSBL_EULERDAMP) != 0;
	result.override_contacts     = (option.enableflags & mjENBL_OVERRIDE) != 0;
	result.energy                = (option.enableflags & mjENBL_ENERGY) != 0;
	result.fwd_inv               = (option.enableflags & mjENBL_FWDINV) != 0;
	result.inv_discrete          = (option.enableflags & mjENBL_INVDISCRETE) != 0;
	result.multiccd              = (option.enableflags & mjENBL_MULTICCD) != 0;
	result.island                = (option.enableflags & mjENBL_ISLAND) != 0;
	return result;
}

RuntimeOptionsPatchResult ParseRuntimeOptionsPatch(const std::vector<RuntimeOptionInput> &input)
{
	RuntimeOptionsPatch patch;
	for (const auto &item : input) {
		const auto type_error = [&item](const char *expected) {
			return RejectPatch(item.field, "expected " + std::string(expected));
		};
		if (item.field == "integrator" || item.field == "cone" || item.field == "jacobian" || item.field == "solver") {
			int value = 0;
			if (!ReadInteger(item, value))
				return type_error("integer");
			if (item.field == "integrator")
				patch.integrator = value;
			else if (item.field == "cone")
				patch.cone = value;
			else if (item.field == "jacobian")
				patch.jacobian = value;
			else
				patch.solver = value;
			continue;
		}
		if (item.field == "iterations" || item.field == "ls_iterations" || item.field == "ls_iter" ||
		    item.field == "noslip_iterations" || item.field == "noslip_iter" || item.field == "ccd_iterations" ||
		    item.field == "ccd_iter" || item.field == "sdf_iterations" || item.field == "sdf_iter" ||
		    item.field == "sdf_initpoints" || item.field == "sdf_init") {
			int value = 0;
			if (!ReadInteger(item, value))
				return type_error("integer");
			if (item.field == "iterations")
				patch.iterations = value;
			else if (item.field == "ls_iterations" || item.field == "ls_iter")
				patch.ls_iterations = value;
			else if (item.field == "noslip_iterations" || item.field == "noslip_iter")
				patch.noslip_iterations = value;
			else if (item.field == "ccd_iterations" || item.field == "ccd_iter")
				patch.ccd_iterations = value;
			else if (item.field == "sdf_iterations" || item.field == "sdf_iter")
				patch.sdf_iterations = value;
			else
				patch.sdf_initpoints = value;
			continue;
		}
		if (item.field == "timestep" || item.field == "tolerance" || item.field == "ls_tolerance" ||
		    item.field == "ls_tol" || item.field == "noslip_tolerance" || item.field == "noslip_tol" ||
		    item.field == "ccd_tolerance" || item.field == "ccd_tol" || item.field == "density" ||
		    item.field == "viscosity" || item.field == "impratio" || item.field == "margin") {
			double value = 0;
			if (!ReadValue(item, value))
				return type_error("double");
			if (item.field == "timestep")
				patch.timestep = value;
			else if (item.field == "tolerance")
				patch.tolerance = value;
			else if (item.field == "ls_tolerance" || item.field == "ls_tol")
				patch.ls_tolerance = value;
			else if (item.field == "noslip_tolerance" || item.field == "noslip_tol")
				patch.noslip_tolerance = value;
			else if (item.field == "ccd_tolerance" || item.field == "ccd_tol")
				patch.ccd_tolerance = value;
			else if (item.field == "density")
				patch.density = value;
			else if (item.field == "viscosity")
				patch.viscosity = value;
			else if (item.field == "impratio")
				patch.impratio = value;
			else
				patch.margin = value;
			continue;
		}
		if (item.field == "gravity" || item.field == "wind" || item.field == "magnetic" || item.field == "solimp" ||
		    item.field == "solref" || item.field == "friction") {
			const auto *value = std::get_if<std::string>(&item.value);
			if (value == nullptr)
				return type_error("space-delimited string");
			std::string reason;
			if (item.field == "gravity") {
				std::array<double, 3> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.gravity = parsed;
			} else if (item.field == "wind") {
				std::array<double, 3> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.wind = parsed;
			} else if (item.field == "magnetic") {
				std::array<double, 3> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.magnetic = parsed;
			} else if (item.field == "solimp") {
				std::array<double, 5> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.solimp = parsed;
			} else if (item.field == "solref") {
				std::array<double, 2> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.solref = parsed;
			} else {
				std::array<double, 5> parsed;
				if (!ParseArray(*value, parsed, reason))
					return RejectPatch(item.field, reason);
				patch.friction = parsed;
			}
			continue;
		}

		const auto set_bool = [&](std::optional<bool> &destination) -> bool {
			bool value = false;
			if (!ReadValue(item, value))
				return false;
			destination = value;
			return true;
		};
#define RUNTIME_OPTION_BOOL(name)      \
	if (item.field == #name) {          \
		if (!set_bool(patch.name))       \
			return type_error("boolean"); \
		continue;                        \
	}
		RUNTIME_OPTION_BOOL(constraint_disabled)
		RUNTIME_OPTION_BOOL(equality_disabled)
		RUNTIME_OPTION_BOOL(frictionloss_disabled)
		RUNTIME_OPTION_BOOL(limit_disabled)
		RUNTIME_OPTION_BOOL(contact_disabled)
		RUNTIME_OPTION_BOOL(passive_disabled)
		RUNTIME_OPTION_BOOL(gravity_disabled)
		RUNTIME_OPTION_BOOL(clampctrl_disabled)
		RUNTIME_OPTION_BOOL(warmstart_disabled)
		RUNTIME_OPTION_BOOL(filterparent_disabled)
		RUNTIME_OPTION_BOOL(actuation_disabled)
		RUNTIME_OPTION_BOOL(refsafe_disabled)
		RUNTIME_OPTION_BOOL(sensor_disabled)
		RUNTIME_OPTION_BOOL(midphase_disabled)
		RUNTIME_OPTION_BOOL(eulerdamp_disabled)
		RUNTIME_OPTION_BOOL(override_contacts)
		RUNTIME_OPTION_BOOL(energy)
		RUNTIME_OPTION_BOOL(fwd_inv)
		RUNTIME_OPTION_BOOL(inv_discrete)
		RUNTIME_OPTION_BOOL(multiccd)
		RUNTIME_OPTION_BOOL(island)
#undef RUNTIME_OPTION_BOOL
		return RejectPatch(item.field, "unknown Runtime Options field");
	}
	return { patch, std::nullopt };
}

RuntimeOptionsValidationResult ValidateRuntimeOptions(const RuntimeOptionsSnapshot &snapshot)
{
	if (snapshot.integrator < 0 || snapshot.integrator > 3)
		return RejectValidation("integrator", "must be in range 0..3");
	if (snapshot.cone < 0 || snapshot.cone > 1)
		return RejectValidation("cone", "must be in range 0..1");
	if (snapshot.jacobian < 0 || snapshot.jacobian > 2)
		return RejectValidation("jacobian", "must be in range 0..2");
	if (snapshot.solver < 0 || snapshot.solver > 2)
		return RejectValidation("solver", "must be in range 0..2");
	if (!Finite(snapshot.timestep) || snapshot.timestep <= 0)
		return RejectValidation("timestep", "must be finite and positive");
	if (snapshot.iterations < 1)
		return RejectValidation("iterations", "must be at least 1");
	if (!Finite(snapshot.tolerance) || snapshot.tolerance < 0)
		return RejectValidation("tolerance", "must be finite and nonnegative");
	if (snapshot.ls_iterations < 1)
		return RejectValidation("ls_iterations", "must be at least 1");
	if (!Finite(snapshot.ls_tolerance) || snapshot.ls_tolerance < 0)
		return RejectValidation("ls_tolerance", "must be finite and nonnegative");
	if (snapshot.noslip_iterations < 0)
		return RejectValidation("noslip_iterations", "must be nonnegative");
	if (!Finite(snapshot.noslip_tolerance) || snapshot.noslip_tolerance < 0)
		return RejectValidation("noslip_tolerance", "must be finite and nonnegative");
	if (snapshot.ccd_iterations < 1)
		return RejectValidation("ccd_iterations", "must be at least 1");
	if (!Finite(snapshot.ccd_tolerance) || snapshot.ccd_tolerance < 0)
		return RejectValidation("ccd_tolerance", "must be finite and nonnegative");
	if (snapshot.sdf_iterations < 1)
		return RejectValidation("sdf_iterations", "must be at least 1");
	if (snapshot.sdf_initpoints < 1)
		return RejectValidation("sdf_initpoints", "must be at least 1");
	if (!Finite(snapshot.density) || snapshot.density < 0)
		return RejectValidation("density", "must be finite and nonnegative");
	if (!Finite(snapshot.viscosity) || snapshot.viscosity < 0)
		return RejectValidation("viscosity", "must be finite and nonnegative");
	if (!Finite(snapshot.impratio) || snapshot.impratio < 0)
		return RejectValidation("impratio", "must be finite and nonnegative");
	if (!Finite(snapshot.margin) || snapshot.margin < 0)
		return RejectValidation("margin", "must be finite and nonnegative");
	for (const auto &[field, values] : std::initializer_list<std::pair<const char *, const std::array<double, 3> *>>{
	         { "gravity", &snapshot.gravity }, { "wind", &snapshot.wind }, { "magnetic", &snapshot.magnetic } }) {
		const auto valid = ValidateFiniteArray(field, *values);
		if (!valid.ok())
			return valid;
	}
	if (const auto valid = ValidateFiniteArray("solimp", snapshot.solimp); !valid.ok())
		return valid;
	if (snapshot.solimp[0] < 0 || snapshot.solimp[0] > 1 || snapshot.solimp[1] < 0 || snapshot.solimp[1] > 1 ||
	    snapshot.solimp[0] > snapshot.solimp[1] || snapshot.solimp[2] < 0 || snapshot.solimp[3] < 0 ||
	    snapshot.solimp[3] > 1 || snapshot.solimp[4] < 1) {
		return RejectValidation("solimp", "must satisfy MuJoCo impedance domains");
	}
	if (const auto valid = ValidateFiniteArray("solref", snapshot.solref); !valid.ok())
		return valid;
	if (snapshot.solref[0] == 0 || snapshot.solref[1] == 0 ||
	    std::signbit(snapshot.solref[0]) != std::signbit(snapshot.solref[1])) {
		return RejectValidation("solref", "time constant and damping ratio must have consistent nonzero signs");
	}
	if (const auto valid = ValidateFiniteArray("friction", snapshot.friction); !valid.ok())
		return valid;
	if (std::any_of(snapshot.friction.begin(), snapshot.friction.end(), [](double value) { return value < 0; })) {
		return RejectValidation("friction", "all values must be nonnegative");
	}
	return {};
}

RuntimeOptionsSnapshot MergeRuntimeOptions(const RuntimeOptionsSnapshot &base, const RuntimeOptionsPatch &patch)
{
	RuntimeOptionsSnapshot result = base;
#define RUNTIME_OPTION_MERGE(name) ApplyOptional(patch.name, result.name)
	RUNTIME_OPTION_MERGE(integrator);
	RUNTIME_OPTION_MERGE(cone);
	RUNTIME_OPTION_MERGE(jacobian);
	RUNTIME_OPTION_MERGE(solver);
	RUNTIME_OPTION_MERGE(timestep);
	RUNTIME_OPTION_MERGE(iterations);
	RUNTIME_OPTION_MERGE(tolerance);
	RUNTIME_OPTION_MERGE(ls_iterations);
	RUNTIME_OPTION_MERGE(ls_tolerance);
	RUNTIME_OPTION_MERGE(noslip_iterations);
	RUNTIME_OPTION_MERGE(noslip_tolerance);
	RUNTIME_OPTION_MERGE(ccd_iterations);
	RUNTIME_OPTION_MERGE(ccd_tolerance);
	RUNTIME_OPTION_MERGE(sdf_iterations);
	RUNTIME_OPTION_MERGE(sdf_initpoints);
	RUNTIME_OPTION_MERGE(density);
	RUNTIME_OPTION_MERGE(viscosity);
	RUNTIME_OPTION_MERGE(impratio);
	RUNTIME_OPTION_MERGE(margin);
	RUNTIME_OPTION_MERGE(gravity);
	RUNTIME_OPTION_MERGE(wind);
	RUNTIME_OPTION_MERGE(magnetic);
	RUNTIME_OPTION_MERGE(solimp);
	RUNTIME_OPTION_MERGE(solref);
	RUNTIME_OPTION_MERGE(friction);
	RUNTIME_OPTION_MERGE(constraint_disabled);
	RUNTIME_OPTION_MERGE(equality_disabled);
	RUNTIME_OPTION_MERGE(frictionloss_disabled);
	RUNTIME_OPTION_MERGE(limit_disabled);
	RUNTIME_OPTION_MERGE(contact_disabled);
	RUNTIME_OPTION_MERGE(passive_disabled);
	RUNTIME_OPTION_MERGE(gravity_disabled);
	RUNTIME_OPTION_MERGE(clampctrl_disabled);
	RUNTIME_OPTION_MERGE(warmstart_disabled);
	RUNTIME_OPTION_MERGE(filterparent_disabled);
	RUNTIME_OPTION_MERGE(actuation_disabled);
	RUNTIME_OPTION_MERGE(refsafe_disabled);
	RUNTIME_OPTION_MERGE(sensor_disabled);
	RUNTIME_OPTION_MERGE(midphase_disabled);
	RUNTIME_OPTION_MERGE(eulerdamp_disabled);
	RUNTIME_OPTION_MERGE(override_contacts);
	RUNTIME_OPTION_MERGE(energy);
	RUNTIME_OPTION_MERGE(fwd_inv);
	RUNTIME_OPTION_MERGE(inv_discrete);
	RUNTIME_OPTION_MERGE(multiccd);
	RUNTIME_OPTION_MERGE(island);
#undef RUNTIME_OPTION_MERGE
	return result;
}

void WriteRuntimeOptions(const RuntimeOptionsSnapshot &snapshot, mjOption &option)
{
	const auto valid = ValidateRuntimeOptions(snapshot);
	if (!valid.ok()) {
		throw std::invalid_argument(valid.error->field + ": " + valid.error->message);
	}
	option.integrator        = snapshot.integrator;
	option.cone              = snapshot.cone;
	option.jacobian          = snapshot.jacobian;
	option.solver            = snapshot.solver;
	option.timestep          = snapshot.timestep;
	option.iterations        = snapshot.iterations;
	option.tolerance         = snapshot.tolerance;
	option.ls_iterations     = snapshot.ls_iterations;
	option.ls_tolerance      = snapshot.ls_tolerance;
	option.noslip_iterations = snapshot.noslip_iterations;
	option.noslip_tolerance  = snapshot.noslip_tolerance;
	option.ccd_iterations    = snapshot.ccd_iterations;
	option.ccd_tolerance     = snapshot.ccd_tolerance;
	option.sdf_iterations    = snapshot.sdf_iterations;
	option.sdf_initpoints    = snapshot.sdf_initpoints;
	option.density           = snapshot.density;
	option.viscosity         = snapshot.viscosity;
	option.impratio          = snapshot.impratio;
	option.o_margin          = snapshot.margin;
	WriteArray(snapshot.gravity, option.gravity);
	WriteArray(snapshot.wind, option.wind);
	WriteArray(snapshot.magnetic, option.magnetic);
	WriteArray(snapshot.solimp, option.o_solimp);
	WriteArray(snapshot.solref, option.o_solref);
	WriteArray(snapshot.friction, option.o_friction);
	option.disableflags = 0;
	SetBit(option.disableflags, mjDSBL_CONSTRAINT, snapshot.constraint_disabled);
	SetBit(option.disableflags, mjDSBL_EQUALITY, snapshot.equality_disabled);
	SetBit(option.disableflags, mjDSBL_FRICTIONLOSS, snapshot.frictionloss_disabled);
	SetBit(option.disableflags, mjDSBL_LIMIT, snapshot.limit_disabled);
	SetBit(option.disableflags, mjDSBL_CONTACT, snapshot.contact_disabled);
	SetBit(option.disableflags, mjDSBL_PASSIVE, snapshot.passive_disabled);
	SetBit(option.disableflags, mjDSBL_GRAVITY, snapshot.gravity_disabled);
	SetBit(option.disableflags, mjDSBL_CLAMPCTRL, snapshot.clampctrl_disabled);
	SetBit(option.disableflags, mjDSBL_WARMSTART, snapshot.warmstart_disabled);
	SetBit(option.disableflags, mjDSBL_FILTERPARENT, snapshot.filterparent_disabled);
	SetBit(option.disableflags, mjDSBL_ACTUATION, snapshot.actuation_disabled);
	SetBit(option.disableflags, mjDSBL_REFSAFE, snapshot.refsafe_disabled);
	SetBit(option.disableflags, mjDSBL_SENSOR, snapshot.sensor_disabled);
	SetBit(option.disableflags, mjDSBL_MIDPHASE, snapshot.midphase_disabled);
	SetBit(option.disableflags, mjDSBL_EULERDAMP, snapshot.eulerdamp_disabled);
	option.enableflags = 0;
	SetBit(option.enableflags, mjENBL_OVERRIDE, snapshot.override_contacts);
	SetBit(option.enableflags, mjENBL_ENERGY, snapshot.energy);
	SetBit(option.enableflags, mjENBL_FWDINV, snapshot.fwd_inv);
	SetBit(option.enableflags, mjENBL_INVDISCRETE, snapshot.inv_discrete);
	SetBit(option.enableflags, mjENBL_MULTICCD, snapshot.multiccd);
	SetBit(option.enableflags, mjENBL_ISLAND, snapshot.island);
}

} // namespace mujoco_ros
