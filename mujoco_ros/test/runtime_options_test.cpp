#include <gtest/gtest.h>

#include <mujoco/mujoco.h>
#include <mujoco_ros/runtime_options.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace mujoco_ros {
namespace {

TEST(RuntimeOptions, StrongGenerationIdsAreNotInterchangeable)
{
	static_assert(!std::is_convertible_v<ModelGeneration, PluginGeneration>);
	static_assert(!std::is_convertible_v<PluginGeneration, FrameGeneration>);
	static_assert(!std::is_convertible_v<FrameGeneration, OptionsEpoch>);
	EXPECT_EQ(ModelGeneration(17).value(), 17u);
	EXPECT_EQ(PluginGeneration(23).value(), 23u);
	EXPECT_EQ(FrameGeneration(29).value(), 29u);
	EXPECT_EQ(OptionsEpoch(31).value(), 31u);
}

TEST(RuntimeOptions, ReadsAndWritesEveryNamedField)
{
	mjOption option;
	mj_defaultOption(&option);
	RuntimeOptionsSnapshot expected;
	expected.integrator            = 3;
	expected.cone                  = 0;
	expected.jacobian              = 1;
	expected.solver                = 2;
	expected.timestep              = 0.002;
	expected.iterations            = 50;
	expected.tolerance             = 2e-8;
	expected.ls_iterations         = 7;
	expected.ls_tolerance          = 3e-4;
	expected.noslip_iterations     = 4;
	expected.noslip_tolerance      = 5e-5;
	expected.ccd_iterations        = 6;
	expected.ccd_tolerance         = 7e-6;
	expected.sdf_iterations        = 8;
	expected.sdf_initpoints        = 9;
	expected.density               = 1.2;
	expected.viscosity             = 0.3;
	expected.impratio              = 1.4;
	expected.margin                = 0.005;
	expected.gravity               = { 1.0, 2.0, 3.0 };
	expected.wind                  = { 4.0, 5.0, 6.0 };
	expected.magnetic              = { 7.0, 8.0, 9.0 };
	expected.solimp                = { 0.1, 0.9, 0.01, 0.5, 2.0 };
	expected.solref                = { 0.02, 1.0 };
	expected.friction              = { 0.1, 0.2, 0.3, 0.4, 0.5 };
	expected.constraint_disabled   = true;
	expected.equality_disabled     = true;
	expected.frictionloss_disabled = true;
	expected.limit_disabled        = true;
	expected.contact_disabled      = true;
	expected.passive_disabled      = true;
	expected.gravity_disabled      = true;
	expected.clampctrl_disabled    = true;
	expected.warmstart_disabled    = true;
	expected.filterparent_disabled = true;
	expected.actuation_disabled    = true;
	expected.refsafe_disabled      = true;
	expected.sensor_disabled       = true;
	expected.midphase_disabled     = true;
	expected.eulerdamp_disabled    = true;
	expected.override_contacts     = true;
	expected.energy                = true;
	expected.fwd_inv               = true;
	expected.inv_discrete          = true;
	expected.multiccd              = true;
	expected.island                = true;

	ASSERT_TRUE(ValidateRuntimeOptions(expected).ok());
	WriteRuntimeOptions(expected, option);
	EXPECT_EQ(ReadRuntimeOptions(option), expected);
}

TEST(RuntimeOptions, InvalidPatchLeavesEveryEffectiveFieldUnchanged)
{
	mjOption option;
	mj_defaultOption(&option);
	const RuntimeOptionsSnapshot before = ReadRuntimeOptions(option);

	const std::vector<RuntimeOptionInput> input = {
		{ "timestep", 0.002 },
		{ "solimp", std::string("0.9 0.95 0.001 0.5 nan") },
	};
	const auto parsed = ParseRuntimeOptionsPatch(input);
	ASSERT_FALSE(parsed.ok());
	ASSERT_TRUE(parsed.error.has_value());
	EXPECT_EQ(parsed.error->field, "solimp");
	EXPECT_EQ(ReadRuntimeOptions(option), before);
}

TEST(RuntimeOptions, ParsesAllTransportKindsAndLegacyArrayNames)
{
	const auto parsed = ParseRuntimeOptionsPatch({
	    { "integrator", std::int64_t(2) },
	    { "timestep", 0.002 },
	    { "iterations", std::int64_t(50) },
	    { "gravity", std::string("1 2 3") },
	    { "constraint_disabled", true },
	});
	ASSERT_TRUE(parsed.ok());
	ASSERT_TRUE(parsed.patch.has_value());
	RuntimeOptionsSnapshot base;
	const auto merged = MergeRuntimeOptions(base, *parsed.patch);
	EXPECT_EQ(merged.integrator, 2);
	EXPECT_DOUBLE_EQ(merged.timestep, 0.002);
	EXPECT_EQ(merged.iterations, 50);
	EXPECT_EQ(merged.gravity, (std::array<double, 3>{ 1, 2, 3 }));
	EXPECT_TRUE(merged.constraint_disabled);
}

TEST(RuntimeOptions, RejectsInvalidDomainsWithFieldSpecificErrors)
{
	const std::vector<std::pair<std::string, RuntimeOptionsSnapshot>> cases = {
		{ "integrator",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.integrator = 4;
		     return value;
		  }() },
		{ "iterations",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.iterations = 0;
		     return value;
		  }() },
		{ "gravity",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.gravity[1] = std::numeric_limits<double>::quiet_NaN();
		     return value;
		  }() },
		{ "solref",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.solref = { -0.02, 1.0 };
		     return value;
		  }() },
		{ "solimp",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.solimp[0] = 1.1;
		     return value;
		  }() },
		{ "friction",
		  [] {
		     RuntimeOptionsSnapshot value;
		     value.friction[0] = -0.1;
		     return value;
		  }() },
	};
	for (const auto &[field, value] : cases) {
		const auto result = ValidateRuntimeOptions(value);
		ASSERT_FALSE(result.ok()) << field;
		ASSERT_TRUE(result.error.has_value());
		EXPECT_EQ(result.error->field, field);
	}
}

TEST(RuntimeOptions, RejectsMalformedArrayLengthsAndNonFiniteScalars)
{
	for (const auto &input : std::vector<RuntimeOptionInput>{
	         { "gravity", std::string("0 0") },
	         { "wind", std::string("0 0 0 0") },
	         { "magnetic", std::string("0 0 nan") },
	         { "solimp", std::string("0.9 0.95 0.001 0.5") },
	         { "solref", std::string("0.02") },
	         { "friction", std::string("1 1 0.005 0.0001") },
	     }) {
		const auto result = ParseRuntimeOptionsPatch({ input });
		EXPECT_FALSE(result.ok());
		EXPECT_TRUE(result.error.has_value());
	}

	RuntimeOptionsSnapshot non_finite;
	non_finite.timestep   = std::numeric_limits<double>::infinity();
	const auto validation = ValidateRuntimeOptions(non_finite);
	EXPECT_FALSE(validation.ok());
	ASSERT_TRUE(validation.error.has_value());
	EXPECT_EQ(validation.error->field, "timestep");
}

TEST(RuntimeOptions, WriteRejectsInvalidCandidateBeforeMutatingOption)
{
	mjOption option;
	mj_defaultOption(&option);
	const mjOption before          = option;
	RuntimeOptionsSnapshot invalid = ReadRuntimeOptions(option);
	invalid.solref                 = { -0.02, 1.0 };

	EXPECT_THROW(WriteRuntimeOptions(invalid, option), std::invalid_argument);
	EXPECT_EQ(std::memcmp(&option, &before, sizeof(mjOption)), 0);
}

TEST(RuntimeOptions, RejectsUnknownFieldsAndWrongTransportTypes)
{
	for (const auto &input : std::vector<RuntimeOptionInput>{
	         { "not_an_option", true },
	         { "timestep", true },
	         { "gravity", 1.0 },
	     }) {
		const auto result = ParseRuntimeOptionsPatch({ input });
		EXPECT_FALSE(result.ok());
		EXPECT_TRUE(result.error.has_value());
	}
}

} // namespace
} // namespace mujoco_ros
