#include <gtest/gtest.h>

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/detail/i_control_backend.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/middleware/middleware_core.hpp>
#include <mujoco_ros/plugin_host.hpp>

#include <memory>
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

TEST(MiddlewareCore, UnattachedPortsFailLoud)
{
	FakeControlBackend backend;
	MiddlewareCore core(backend);
	EXPECT_THROW(core.config_source(), std::logic_error);
	EXPECT_THROW(core.time_publisher(), std::logic_error);

	FakeConfigSource config;
	FakeTimePublisher time_pub;
	core.AttachPorts(config, time_pub);
	EXPECT_EQ(&core.config_source(), &config);
	EXPECT_EQ(&core.time_publisher(), &time_pub);
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

// --- Plugin Host owned by the Core (no MujocoEnv anywhere in this translation unit) ---

struct CountingPluginState
{
	int controls = 0;
};

class CountingAdapter : public IPluginAdapter
{
public:
	explicit CountingAdapter(int *live) : live_(live) { ++*live_; }
	~CountingAdapter() override { --*live_; }

	const std::string &Name() const override { return name_; }
	const std::string &Type() const override { return type_; }
	bool Load(const mjModel *, mjData *, std::string &) override { return true; }
	void Control(const mjModel *, mjData *) override { ++state_.controls; }
	void Passive(const mjModel *, mjData *) override {}
	void Render(const mjModel *, mjData *, mjvScene *) override {}
	void LastStage(const mjModel *, mjData *) override {}
	void Reset() override {}
	void GeometryChanged(const mjModel *, mjData *, int) override {}
	PluginStat Statistics() const override
	{
		PluginStat stat;
		stat.name = name_;
		stat.type = type_;
		return stat;
	}
	void *BackendObject() override { return &state_; }

private:
	int *live_;
	CountingPluginState state_;
	std::string name_ = "counter";
	std::string type_ = "test/CountingAdapter";
};

class CountingFactory final : public IPluginAdapterFactory
{
public:
	std::vector<std::unique_ptr<IPluginAdapter>> CreateAdapters() override
	{
		++creations;
		std::vector<std::unique_ptr<IPluginAdapter>> adapters;
		adapters.push_back(std::make_unique<CountingAdapter>(&live));
		return adapters;
	}
	int live      = 0;
	int creations = 0;
};

TEST(MiddlewareCore, RebindPluginsBumpsPluginGenerationAndDropsCrossGenerationState)
{
	FakeControlBackend backend;
	CountingFactory factory;
	MiddlewareCore core(backend);
	core.AttachPluginFactory(factory);

	const auto first = core.RebindPlugins(nullptr, nullptr, ModelGeneration(1));
	EXPECT_EQ(first.generation.value(), 1u);
	EXPECT_EQ(first.model_generation.value(), 1u);
	EXPECT_TRUE(first.failures.empty());
	ASSERT_EQ(core.GetPluginHandles().size(), 1u);
	const auto old_handle = core.GetPluginHandles().front();

	core.DispatchControl(ModelGeneration(1), nullptr, nullptr);
	EXPECT_EQ(old_handle.WithBackend<CountingPluginState>([](CountingPluginState &s) { return s.controls; }), 1);

	const auto second = core.RebindPlugins(nullptr, nullptr, ModelGeneration(2));
	EXPECT_EQ(second.generation.value(), 2u);
	EXPECT_EQ(core.plugin_generation().value(), 2u);
	EXPECT_EQ(factory.creations, 2);
	EXPECT_EQ(factory.live, 1) << "old Plugin Generation must be destroyed before the new one is active";

	// Old handle is rejected; the new generation starts from fresh plugin state.
	EXPECT_THROW(old_handle.WithAccess([](const ScopedPluginAccess &) {}), std::runtime_error);
	const auto new_handles = core.GetPluginHandles();
	ASSERT_EQ(new_handles.size(), 1u);
	EXPECT_EQ(new_handles.front().Generation().value(), 2u);
	EXPECT_EQ(new_handles.front().WithBackend<CountingPluginState>([](CountingPluginState &s) { return s.controls; }),
	          0);

	// Stale Model Generation callbacks are refused by the Core-hosted Plugin Host.
	EXPECT_THROW(core.DispatchControl(ModelGeneration(1), nullptr, nullptr), std::runtime_error);

	core.QuiescePlugins();
	EXPECT_EQ(factory.live, 0);
	EXPECT_EQ(core.ReadyPluginCount(), 0u);
}

TEST(MiddlewareCore, PluginApiWithoutFactoryFailsLoud)
{
	FakeControlBackend backend;
	MiddlewareCore core(backend);
	EXPECT_THROW(core.RebindPlugins(nullptr, nullptr, ModelGeneration(1)), std::logic_error);
	EXPECT_THROW(core.WithPluginAccess([](const ScopedPluginAccess &) {}), std::runtime_error);
}

TEST(MiddlewareCore, PendingRuntimeOptionsReapplyOnceToReboundModel)
{
	FakeControlBackend backend;
	mj_defaultOption(&backend.option);
	MiddlewareCore core(backend);

	RuntimeOptionsPatch patch;
	patch.timestep = 0.004;
	core.SetPendingRuntimeOptionsPatch(patch);
	const auto epoch = core.options_epoch().value();

	mjOption fresh;
	mj_defaultOption(&fresh);
	EXPECT_FALSE(core.ApplyPendingRuntimeOptions(fresh).has_value());
	EXPECT_DOUBLE_EQ(fresh.timestep, 0.004);
	EXPECT_EQ(core.options_epoch().value(), epoch + 1);

	// consumed: a later model generation does not receive it again
	mjOption next;
	mj_defaultOption(&next);
	EXPECT_FALSE(core.ApplyPendingRuntimeOptions(next).has_value());
	EXPECT_DOUBLE_EQ(next.timestep, 0.002);
	EXPECT_EQ(core.options_epoch().value(), epoch + 1);
}

} // namespace
} // namespace mujoco_ros::middleware
