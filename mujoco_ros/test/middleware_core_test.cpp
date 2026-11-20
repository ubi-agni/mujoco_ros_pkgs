#include <gtest/gtest.h>

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/detail/i_control_backend.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/middleware/middleware_core.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mujoco_ros::middleware {
namespace {

class FakeConfigSource : public IConfigSource
{
public:
	std::optional<bool> GetBool(const std::string &) const override { return std::nullopt; }
	std::optional<int> GetInt(const std::string &) const override { return std::nullopt; }
	std::optional<double> GetDouble(const std::string &) const override { return std::nullopt; }
	std::optional<std::string> GetString(const std::string &) const override { return std::nullopt; }
	std::optional<std::vector<std::string>> GetStringList(const std::string &) const override { return std::nullopt; }
};

class FakeTimePublisher : public ITimePublisher
{
public:
	void PublishSimTime(mjtNum sim_time) override
	{
		last = sim_time;
		++count;
	}
	mjtNum last = -1.0;
	int count   = 0;
};

class FakeControlBackend : public IControlBackend
{
public:
	void ControlWithPhysicsLock(const std::function<void()> &func) override { func(); }
	std::uint64_t ControlAdmissionEpoch() const override { return admission_epoch; }
	SimulationControlSnapshot ControlSnapshot() const override
	{
		SimulationControlSnapshot snapshot;
		snapshot.model_lifecycle = lifecycle;
		snapshot.running         = running;
		return snapshot;
	}
	mjOption *ControlModelOption() override { return has_model ? &option : nullptr; }
	bool ControlHasModel() const override { return has_model; }
	void ControlNotifySettingsChanged() override { ++settings_changed; }
	bool ControlVerifyAdminHash(const std::string &hash) override { return hash == admin_hash; }
	void ControlSetPaused(bool paused, bool) override { running = !paused; }
	void ControlSetAdminHash(const std::string &hash) override { admin_hash = hash; }
	bool ControlOnPhysicsThread() const override { return false; }
	bool ControlWaitPhysicsQuiescent() override { return true; }
	bool ControlAdmitManualSteps(int, ManualStepToken *) override { return false; }
	ManualStepSnapshot ControlManualStepSnapshot(ManualStepToken) const override { return {}; }
	ManualStepSnapshot ControlWaitManualStepUpdate(ManualStepToken, int) const override { return {}; }
	bool ControlCancelManualSteps(ManualStepToken) override { return false; }
	void ControlAcknowledgeManualStep(ManualStepToken) override {}
	void ControlRequestReset() override {}
	void ControlRequestReload() override {}
	int ControlOperationalStatus() override { return 0; }
	std::size_t ControlMaxModelStringLength() const override { return 10; }
	void ControlQueueModelStringLoad(const std::string &) override {}
	bool ControlLoadResult(char *, int) override { return true; }
	void ControlLogDebug(const std::string &) override {}
	void ControlLogWarn(const std::string &) override {}
	void ControlLogError(const std::string &) override {}

	ModelLifecyclePhase lifecycle = ModelLifecyclePhase::kOperational;
	bool has_model                = true;
	bool running                  = true;
	std::uint64_t admission_epoch = 0;
	int settings_changed          = 0;
	std::string admin_hash        = "secret";
	mjOption option;
};

TEST(MiddlewareCore, MiddlewareCoreApplyRuntimeOptionsMatchesEnvContract)
{
	FakeControlBackend backend;
	mj_defaultOption(&backend.option);
	MiddlewareCore core(backend);

	const auto before = core.GetRuntimeOptions();
	ASSERT_TRUE(before.ok());
	const auto start_epoch = before.epoch;

	const auto applied = core.ApplyRuntimeOptions({ { "timestep", 0.002 } });
	ASSERT_TRUE(applied.ok());
	EXPECT_EQ(applied.epoch.value(), start_epoch.value() + 1);
	EXPECT_DOUBLE_EQ(backend.option.timestep, 0.002);
	EXPECT_EQ(backend.settings_changed, 1);

	// Invalid patch: rejected, epoch preserved, model untouched.
	const auto invalid = core.ApplyRuntimeOptions({ { "timestep", -1.0 } });
	EXPECT_FALSE(invalid.ok());
	EXPECT_EQ(invalid.epoch.value(), applied.epoch.value());
	EXPECT_DOUBLE_EQ(backend.option.timestep, 0.002);

	// Loading Window: same reject string as MujocoEnv, epoch preserved.
	backend.lifecycle  = ModelLifecyclePhase::kLoading;
	const auto loading = core.ApplyRuntimeOptions({ { "timestep", 0.004 } });
	ASSERT_FALSE(loading.ok());
	EXPECT_EQ(loading.error->message, "Runtime Options unavailable during Loading Window");
	EXPECT_EQ(loading.error->field, "");
	EXPECT_EQ(loading.epoch.value(), applied.epoch.value());
	EXPECT_FALSE(core.GetRuntimeOptions().ok());
	EXPECT_DOUBLE_EQ(backend.option.timestep, 0.002);
}

TEST(MiddlewareCore, TogglePausedKeepsAdminHashRule)
{
	FakeControlBackend backend;
	MiddlewareCore core(backend);
	EXPECT_FALSE(core.TogglePaused(true, "wrong"));
	EXPECT_TRUE(backend.running);
	EXPECT_TRUE(core.TogglePaused(true, "secret"));
	EXPECT_FALSE(backend.running);
	EXPECT_TRUE(core.TogglePaused(false, "wrong")); // unpausing is not gated
	EXPECT_TRUE(backend.running);
}

TEST(MiddlewareCore, ControlApiWithoutBackendFailsLoud)
{
	FakeConfigSource config;
	FakeTimePublisher time_pub;
	MiddlewareCore core(config, time_pub);
	EXPECT_THROW(core.ApplyRuntimeOptions({}), std::logic_error);
}

TEST(MiddlewareCore, MiddlewareCoreConstructsWithFakePorts)
{
	FakeConfigSource config;
	FakeTimePublisher time_pub;
	MiddlewareCore core(config, time_pub);
	EXPECT_EQ(&core.config_source(), &config);
	EXPECT_EQ(&core.time_publisher(), &time_pub);
	EXPECT_EQ(time_pub.count, 0);
}

} // namespace
} // namespace mujoco_ros::middleware
