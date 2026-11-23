#pragma once

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/detail/i_control_backend.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/plugin_adapter.hpp>
#include <mujoco_ros/plugin_host.hpp>
#include <mujoco_ros/runtime_options.hpp>
#include <mujoco_ros/simulation_control_state.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace mujoco_ros::middleware {

struct ManualStepRequest
{
	bool accepted         = false;
	ManualStepToken token = 0;
};

/**
 * Simulation Middleware Core.
 *
 * Flavor-agnostic; talks to the outside only through IConfigSource, ITimePublisher and (for the
 * Simulation Control API) IControlBackend. Does not own the ports. Hosts the Simulation Control API and
 * Runtime Options apply/get (ADR-0005 rules: Loading Window, admin_hash), the Plugin Host (explore/load/
 * execute/dispatch via an injected IPluginAdapterFactory) and the state-preserving rebind hooks that run on a
 * model generation change:
 *   1. RebindPlugins(): quiesce the old Plugin Generation, bump the Plugin Generation, rebuild every plugin by
 *      name through the factory (name-keyed rebind: no plugin instance or state survives a generation).
 *   2. ApplyPendingRuntimeOptions(): re-apply the pending Runtime Options patch to the new model's options.
 * The Core never holds a MujocoEnv: physics-lock exclusion goes through IControlBackend::ControlWithPhysicsLock
 * and the plugin factory is injected, so a P2 host (Studio) can rehost the Core with its own backend.
 *
 * Residual (documented, not in P1): `mj_copyBack` of a live mjSpec is not performed; the P1 reload recompiles
 * from source and the spec is owned by the host. P2 (Studio owns mjSpec) wires it into the same hook.
 * Residual physics-only callbacks: the factory the flavor injects still hands plugins a MujocoEnv* (the
 * plugin ABI), and physics stepping stays on MujocoEnv.
 */
class MiddlewareCore
{
public:
	MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher);
	MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher, IControlBackend &control_backend);
	/// Control-only construction; config/time ports MUST be attached via AttachPorts() before use,
	/// otherwise config_source()/time_publisher() throw std::logic_error.
	explicit MiddlewareCore(IControlBackend &control_backend);
	~MiddlewareCore();

	MiddlewareCore(const MiddlewareCore &)            = delete;
	MiddlewareCore &operator=(const MiddlewareCore &) = delete;

	IConfigSource &config_source() const;
	ITimePublisher &time_publisher() const;
	/// Replaces the (inert) default ports with the flavor adapters. Ports must outlive the Core.
	void AttachPorts(IConfigSource &config_source, ITimePublisher &time_publisher);

	// --- Simulation Control API (throws std::logic_error when no control backend is attached) ---
	SimulationControlSnapshot GetControlSnapshot() const;
	bool TogglePaused(bool paused, const std::string &admin_hash = std::string(), bool blocking = false);
	/// Parameter-driven pause (no unauthorized-pause check; callers are the parameter/reconfigure surfaces).
	void SetPaused(bool paused, bool notify_settings_changed = true);
	void SetAdminHash(const std::string &admin_hash);
	bool Step(int num_steps = 1, bool blocking = true);
	bool RequestManualSteps(int num_steps);
	ManualStepRequest RequestManualStepsWithToken(int num_steps);
	ManualStepSnapshot GetManualStepSnapshot(ManualStepToken token) const;
	ManualStepSnapshot WaitForManualStepUpdate(ManualStepToken token, int observed_pending_steps) const;
	bool CancelManualSteps(ManualStepToken token = 0);
	void AcknowledgeManualStep(ManualStepToken token);
	void Reset();
	void RequestReset();
	void RequestReload();
	bool LoadModelFromString(const std::string &model, char *load_error = nullptr, int error_sz = 0);

	// --- Runtime Options ---
	RuntimeOptionsTransactionResult GetRuntimeOptions();
	RuntimeOptionsTransactionResult ApplyRuntimeOptions(const std::vector<RuntimeOptionInput> &input);
	OptionsEpoch options_epoch() const { return options_epoch_; }
	/// Bumps the epoch for a Runtime Options write performed outside ApplyRuntimeOptions (startup patch).
	OptionsEpoch AdvanceOptionsEpoch();
	/// Startup Runtime Options patch, applied once to the next model generation (rebind hook).
	void SetPendingRuntimeOptionsPatch(const RuntimeOptionsPatch &patch);
	void ClearPendingRuntimeOptions();
	bool HasPendingRuntimeOptions() const { return has_pending_runtime_options_; }
	/// Applies and consumes the pending patch on `option` (bumps the epoch). Returns the validation error when
	/// the patch is rejected (patch is dropped either way). No-op without a pending patch.
	std::optional<RuntimeOptionsError> ApplyPendingRuntimeOptions(mjOption &option);

	// --- Plugin Host (explore/load/execute/dispatch) ---
	/// Creates the Plugin Host over the flavor-private factory. The factory must outlive the Core.
	void AttachPluginFactory(IPluginAdapterFactory &factory);
	bool HasPluginHost() const { return plugin_host_ != nullptr; }
	/// Rebind hook for a model generation change: new Plugin Generation, fresh plugins by name.
	/// Throws std::logic_error when no factory is attached.
	PluginLoadReport RebindPlugins(const mjModel *model, mjData *data, ModelGeneration model_generation);
	/// Destroys the active Plugin Generation. No-op without a Plugin Host.
	void QuiescePlugins();
	/// Invalidates outstanding PluginHandles (host teardown).
	void InvalidatePluginHandles();
	PluginGeneration plugin_generation() const { return plugin_generation_; }

	void DispatchControl(ModelGeneration, const mjModel *, mjData *);
	void DispatchPassive(ModelGeneration, const mjModel *, mjData *);
	void DispatchRender(ModelGeneration, const mjModel *, mjData *, mjvScene *);
	void DispatchLastStage(ModelGeneration, const mjModel *, mjData *);
	void ResetPlugins(ModelGeneration);
	void NotifyPluginGeometryChanged(ModelGeneration, const mjModel *, mjData *, int geom_id);

	std::vector<PluginStat> PluginStatistics() const;
	std::size_t ReadyPluginCount() const;
	/// Throws std::runtime_error when no Plugin Generation is active.
	ModelGeneration ActivePluginModelGeneration() const;
	std::vector<PluginLoadFailure> PluginDiagnostics() const;
	std::vector<PluginHandle> GetPluginHandles() const;

	/// Runs `func(const ScopedPluginAccess&)` under the physics lock; the access is generation checked.
	template <typename Func>
	decltype(auto) WithPluginAccess(Func &&func) const
	{
		return RunWithPluginHost([&](PluginHost &host) { return host.AcquireScopedAccess(); }, std::forward<Func>(func));
	}
	template <typename Func>
	decltype(auto) WithPluginAccess(PluginGeneration expected_generation, Func &&func) const
	{
		return RunWithPluginHost([&](PluginHost &host) { return host.AcquireScopedAccess(expected_generation); },
		                         std::forward<Func>(func));
	}
	template <typename Func>
	decltype(auto) WithPluginAccess(ModelGeneration expected_model_generation, PluginGeneration expected_generation,
	                                Func &&func) const
	{
		return RunWithPluginHost(
		    [&](PluginHost &host) { return host.AcquireScopedAccess(expected_model_generation, expected_generation); },
		    std::forward<Func>(func));
	}

private:
	PluginHost &RequirePluginHost() const;

	template <typename Acquire, typename Func>
	auto RunWithPluginHost(Acquire &&acquire, Func &&func) const
	{
		using Result  = std::decay_t<std::invoke_result_t<Func, const ScopedPluginAccess &>>;
		auto &backend = Backend();
		if constexpr (std::is_void_v<Result>) {
			backend.ControlWithPhysicsLock([&]() {
				auto access = acquire(RequirePluginHost());
				std::invoke(func, access);
			});
		} else {
			std::optional<Result> result;
			backend.ControlWithPhysicsLock([&]() {
				auto access = acquire(RequirePluginHost());
				result.emplace(std::invoke(func, access));
			});
			return std::move(*result);
		}
	}

	IControlBackend &Backend() const;

	IConfigSource *config_source_     = nullptr;
	ITimePublisher *time_publisher_   = nullptr;
	IControlBackend *control_backend_ = nullptr;
	OptionsEpoch options_epoch_;
	RuntimeOptionsPatch pending_runtime_options_;
	bool has_pending_runtime_options_ = false;

	// Plugin Host. Declared last so plugins are destroyed first.
	IPluginAdapterFactory *plugin_factory_ = nullptr;
	std::shared_ptr<void> plugin_lifetime_ = std::make_shared<int>(0);
	PluginGeneration plugin_generation_;
	std::unique_ptr<PluginHost> plugin_host_;
};

} // namespace mujoco_ros::middleware

namespace mujoco_ros {

template <typename Func>
decltype(auto) PluginHandle::WithAccess(Func &&func) const
{
	if (core_ == nullptr || lifetime_.expired()) {
		throw std::runtime_error("plugin handle belongs to a destroyed Middleware Core");
	}
	return core_->WithPluginAccess(model_generation_, generation_, std::forward<Func>(func));
}

template <typename Backend, typename Func>
decltype(auto) PluginHandle::WithBackend(Func &&func) const
{
	return WithAccess([this, &func](const ScopedPluginAccess &access) -> decltype(auto) {
		auto *backend = access.BackendObject<Backend>(name_, type_);
		return std::invoke(std::forward<Func>(func), *backend);
	});
}

} // namespace mujoco_ros
