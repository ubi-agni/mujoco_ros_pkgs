#pragma once

#include <mujoco_ros/generation.hpp>
#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/detail/i_control_backend.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/runtime_options.hpp>
#include <mujoco_ros/simulation_control_state.hpp>

#include <memory>
#include <string>
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
 * Runtime Options apply/get (ADR-0005 rules: Loading Window, admin_hash). The Plugin Host moves in later.
 */
class MiddlewareCore
{
public:
	MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher);
	MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher, IControlBackend &control_backend);
	/// Control-only construction; config/time ports are inert until the flavor adapters land.
	explicit MiddlewareCore(IControlBackend &control_backend);
	~MiddlewareCore();

	MiddlewareCore(const MiddlewareCore &)            = delete;
	MiddlewareCore &operator=(const MiddlewareCore &) = delete;

	IConfigSource &config_source() const { return config_source_; }
	ITimePublisher &time_publisher() const { return time_publisher_; }

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

private:
	IControlBackend &Backend() const;

	std::unique_ptr<IConfigSource> owned_config_source_;
	std::unique_ptr<ITimePublisher> owned_time_publisher_;
	IConfigSource &config_source_;
	ITimePublisher &time_publisher_;
	IControlBackend *control_backend_ = nullptr;
	OptionsEpoch options_epoch_;
};

} // namespace mujoco_ros::middleware
