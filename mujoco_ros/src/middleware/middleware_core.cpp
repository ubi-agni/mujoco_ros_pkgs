#include <mujoco_ros/middleware/middleware_core.hpp>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <thread>

namespace mujoco_ros::middleware {

namespace {

constexpr const char *kLoadingWindowMessage = "Runtime Options unavailable during Loading Window";

} // namespace

MiddlewareCore::MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher)
    : config_source_(&config_source), time_publisher_(&time_publisher)
{
}

MiddlewareCore::MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher,
                               IControlBackend &control_backend)
    : config_source_(&config_source), time_publisher_(&time_publisher), control_backend_(&control_backend)
{
}

MiddlewareCore::MiddlewareCore(IControlBackend &control_backend) : control_backend_(&control_backend) {}

MiddlewareCore::~MiddlewareCore() = default;

void MiddlewareCore::AttachPorts(IConfigSource &config_source, ITimePublisher &time_publisher)
{
	config_source_  = &config_source;
	time_publisher_ = &time_publisher;
}

IConfigSource &MiddlewareCore::config_source() const
{
	if (config_source_ == nullptr) {
		throw std::logic_error("MiddlewareCore: no config source attached");
	}
	return *config_source_;
}

ITimePublisher &MiddlewareCore::time_publisher() const
{
	if (time_publisher_ == nullptr) {
		throw std::logic_error("MiddlewareCore: no time publisher attached");
	}
	return *time_publisher_;
}

IControlBackend &MiddlewareCore::Backend() const
{
	if (control_backend_ == nullptr) {
		throw std::logic_error("MiddlewareCore: no control backend attached");
	}
	return *control_backend_;
}

SimulationControlSnapshot MiddlewareCore::GetControlSnapshot() const
{
	return Backend().ControlSnapshot();
}

bool MiddlewareCore::TogglePaused(bool paused, const std::string &admin_hash, bool blocking)
{
	auto &backend = Backend();
	backend.ControlLogDebug("Trying to toggle pause");
	if (paused && !backend.ControlVerifyAdminHash(admin_hash)) {
		backend.ControlLogError("Unauthorized pause request detected. Ignoring request");
		return false;
	}
	backend.ControlSetPaused(paused, true);
	if (!blocking || !paused) {
		return true;
	}
	if (backend.ControlOnPhysicsThread()) {
		backend.ControlLogWarn("Cannot block for pause on the physics thread");
		return false;
	}
	if (!backend.ControlWaitPhysicsQuiescent()) {
		backend.ControlLogError("Timed out waiting for physics to acknowledge pause");
		return false;
	}
	return true;
}

void MiddlewareCore::SetPaused(bool paused, bool notify_settings_changed)
{
	Backend().ControlSetPaused(paused, notify_settings_changed);
}

void MiddlewareCore::SetAdminHash(const std::string &admin_hash)
{
	Backend().ControlSetAdminHash(admin_hash);
}

bool MiddlewareCore::Step(int num_steps, bool blocking)
{
	auto &backend       = Backend();
	const auto snapshot = backend.ControlSnapshot();
	if (snapshot.shutdown_requested) {
		backend.ControlLogWarn("Step requested after shutdown has been requested. Ignoring step request.");
		return false;
	}
	if (!backend.ControlHasModel()) {
		backend.ControlLogError("No model loaded. Cannot step");
		return false;
	}
	if (snapshot.running) {
		backend.ControlLogWarn("Simulation is already running. Ignoring request");
		return false;
	}
	if (num_steps <= 0) {
		backend.ControlLogWarn("Number of steps must be positive. Ignoring request");
		return false;
	}
	if (blocking && backend.ControlOnPhysicsThread()) {
		backend.ControlLogWarn("Simulation is running in the same thread. Cannot block! Ignoring request");
		return false;
	}
	backend.ControlLogDebug("Handling request of stepping " + std::to_string(num_steps) + " steps");
	if (!RequestManualSteps(num_steps)) {
		return false;
	}
	if (blocking) {
		backend.ControlLogDebug("\t blocking until steps are done");
		while (backend.ControlSnapshot().pending_steps > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
	return true;
}

bool MiddlewareCore::RequestManualSteps(int num_steps)
{
	return Backend().ControlAdmitManualSteps(num_steps, nullptr);
}

ManualStepRequest MiddlewareCore::RequestManualStepsWithToken(int num_steps)
{
	ManualStepToken token = 0;
	if (!Backend().ControlAdmitManualSteps(num_steps, &token)) {
		return ManualStepRequest{};
	}
	return ManualStepRequest{ true, token };
}

ManualStepSnapshot MiddlewareCore::GetManualStepSnapshot(ManualStepToken token) const
{
	return Backend().ControlManualStepSnapshot(token);
}

ManualStepSnapshot MiddlewareCore::WaitForManualStepUpdate(ManualStepToken token, int observed_pending_steps) const
{
	return Backend().ControlWaitManualStepUpdate(token, observed_pending_steps);
}

bool MiddlewareCore::CancelManualSteps(ManualStepToken token)
{
	return Backend().ControlCancelManualSteps(token);
}

void MiddlewareCore::AcknowledgeManualStep(ManualStepToken token)
{
	Backend().ControlAcknowledgeManualStep(token);
}

void MiddlewareCore::RequestReset()
{
	Backend().ControlRequestReset();
}

void MiddlewareCore::RequestReload()
{
	Backend().ControlRequestReload();
}

void MiddlewareCore::Reset()
{
	auto &backend = Backend();
	backend.ControlLogDebug("Reset requested");
	backend.ControlRequestReset();
	while (backend.ControlOperationalStatus() > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

bool MiddlewareCore::LoadModelFromString(const std::string &model, char *load_error, int error_sz)
{
	auto &backend = Backend();
	backend.ControlLogDebug("Loading model from string requested");
	if (model.size() > backend.ControlMaxModelStringLength()) {
		backend.ControlLogError(
		    "Model string is too long. Max length: " + std::to_string(backend.ControlMaxModelStringLength()) + " (got " +
		    std::to_string(model.size()) + ")");
		return false;
	}
	backend.ControlLogDebug("Issuing model load");
	backend.ControlQueueModelStringLoad(model);
	while (backend.ControlOperationalStatus() > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return backend.ControlLoadResult(load_error, error_sz);
}

RuntimeOptionsTransactionResult MiddlewareCore::GetRuntimeOptions()
{
	auto &backend              = Backend();
	const auto admission_epoch = backend.ControlAdmissionEpoch();
	std::optional<RuntimeOptionsTransactionResult> result;
	backend.ControlWithPhysicsLock([&]() {
		const auto lifecycle = backend.ControlSnapshot().model_lifecycle;
		mjOption *option     = backend.ControlModelOption();
		if (lifecycle != ModelLifecyclePhase::kOperational || option == nullptr ||
		    admission_epoch != backend.ControlAdmissionEpoch()) {
			result = RuntimeOptionsTransactionResult::Rejected({ "", kLoadingWindowMessage }, options_epoch_);
			return;
		}
		result = RuntimeOptionsTransactionResult::Applied(ReadRuntimeOptions(*option), options_epoch_);
	});
	return *result;
}

RuntimeOptionsTransactionResult MiddlewareCore::ApplyRuntimeOptions(const std::vector<RuntimeOptionInput> &input)
{
	auto &backend              = Backend();
	const auto admission_epoch = backend.ControlAdmissionEpoch();
	std::optional<RuntimeOptionsTransactionResult> result;
	backend.ControlWithPhysicsLock([&]() {
		const auto lifecycle = backend.ControlSnapshot().model_lifecycle;
		mjOption *option     = backend.ControlModelOption();
		if (lifecycle != ModelLifecyclePhase::kOperational || option == nullptr ||
		    admission_epoch != backend.ControlAdmissionEpoch()) {
			result = RuntimeOptionsTransactionResult::Rejected({ "", kLoadingWindowMessage }, options_epoch_);
			return;
		}
		const auto parsed = ParseRuntimeOptionsPatch(input);
		if (!parsed.ok()) {
			result = RuntimeOptionsTransactionResult::Rejected(*parsed.error, options_epoch_);
			return;
		}
		const auto candidate = MergeRuntimeOptions(ReadRuntimeOptions(*option), *parsed.patch);
		const auto valid     = ValidateRuntimeOptions(candidate);
		if (!valid.ok()) {
			result = RuntimeOptionsTransactionResult::Rejected(*valid.error, options_epoch_);
			return;
		}
		mjOption candidate_option = *option;
		WriteRuntimeOptions(candidate, candidate_option);
		*option = candidate_option;
		backend.ControlNotifySettingsChanged();
		options_epoch_ = OptionsEpoch(options_epoch_.value() + 1);
		result         = RuntimeOptionsTransactionResult::Applied(candidate, options_epoch_);
	});
	return *result;
}

OptionsEpoch MiddlewareCore::AdvanceOptionsEpoch()
{
	options_epoch_ = OptionsEpoch(options_epoch_.value() + 1);
	return options_epoch_;
}

void MiddlewareCore::SetPendingRuntimeOptionsPatch(const RuntimeOptionsPatch &patch)
{
	pending_runtime_options_     = patch;
	has_pending_runtime_options_ = true;
}

void MiddlewareCore::ClearPendingRuntimeOptions()
{
	pending_runtime_options_     = {};
	has_pending_runtime_options_ = false;
}

std::optional<RuntimeOptionsError> MiddlewareCore::ApplyPendingRuntimeOptions(mjOption &option)
{
	if (!has_pending_runtime_options_) {
		return std::nullopt;
	}
	const auto patch = pending_runtime_options_;
	ClearPendingRuntimeOptions();
	const auto candidate = MergeRuntimeOptions(ReadRuntimeOptions(option), patch);
	const auto valid     = ValidateRuntimeOptions(candidate);
	if (!valid.ok()) {
		return valid.error;
	}
	mjOption candidate_option = option;
	WriteRuntimeOptions(candidate, candidate_option);
	option = candidate_option;
	AdvanceOptionsEpoch();
	return std::nullopt;
}

PluginHost &MiddlewareCore::RequirePluginHost() const
{
	if (!plugin_host_) {
		throw std::runtime_error("Plugin Host is not initialized");
	}
	return *plugin_host_;
}

void MiddlewareCore::AttachPluginFactory(IPluginAdapterFactory &factory)
{
	if (plugin_host_) {
		plugin_host_->QuiesceAndDestroy();
	}
	plugin_factory_ = &factory;
	plugin_host_    = std::make_unique<PluginHost>(factory);
}

PluginLoadReport MiddlewareCore::RebindPlugins(const mjModel *model, mjData *data, ModelGeneration model_generation)
{
	if (!plugin_host_) {
		throw std::logic_error("MiddlewareCore: no plugin factory attached");
	}
	// LoadGeneration quiesces the previous Plugin Generation first; the factory then rebuilds plugins by name.
	const auto next    = PluginGeneration(plugin_generation_.value() + 1);
	auto report        = plugin_host_->LoadGeneration(model, data, model_generation, next);
	plugin_generation_ = report.generation;
	return report;
}

void MiddlewareCore::QuiescePlugins()
{
	if (plugin_host_) {
		plugin_host_->QuiesceAndDestroy();
	}
}

void MiddlewareCore::InvalidatePluginHandles()
{
	plugin_lifetime_.reset();
}

void MiddlewareCore::DispatchControl(ModelGeneration generation, const mjModel *model, mjData *data)
{
	RequirePluginHost().DispatchControl(generation, model, data);
}

void MiddlewareCore::DispatchPassive(ModelGeneration generation, const mjModel *model, mjData *data)
{
	RequirePluginHost().DispatchPassive(generation, model, data);
}

void MiddlewareCore::DispatchRender(ModelGeneration generation, const mjModel *model, mjData *data, mjvScene *scene)
{
	RequirePluginHost().DispatchRender(generation, model, data, scene);
}

void MiddlewareCore::DispatchLastStage(ModelGeneration generation, const mjModel *model, mjData *data)
{
	RequirePluginHost().DispatchLastStage(generation, model, data);
}

void MiddlewareCore::ResetPlugins(ModelGeneration generation)
{
	RequirePluginHost().Reset(generation);
}

void MiddlewareCore::NotifyPluginGeometryChanged(ModelGeneration generation, const mjModel *model, mjData *data,
                                                 int geom_id)
{
	RequirePluginHost().NotifyGeometryChanged(generation, model, data, geom_id);
}

std::vector<PluginStat> MiddlewareCore::PluginStatistics() const
{
	std::vector<PluginStat> stats;
	Backend().ControlWithPhysicsLock([&]() { stats = RequirePluginHost().Statistics(); });
	return stats;
}

std::size_t MiddlewareCore::ReadyPluginCount() const
{
	return plugin_host_ ? plugin_host_->ReadyCount() : 0;
}

ModelGeneration MiddlewareCore::ActivePluginModelGeneration() const
{
	return RequirePluginHost().ActiveModelGeneration();
}

std::vector<PluginLoadFailure> MiddlewareCore::PluginDiagnostics() const
{
	return plugin_host_ ? plugin_host_->Diagnostics() : std::vector<PluginLoadFailure>{};
}

std::vector<PluginHandle> MiddlewareCore::GetPluginHandles() const
{
	return WithPluginAccess([this](const ScopedPluginAccess &access) {
		std::vector<PluginHandle> handles;
		for (const auto &stat : access.Statistics()) {
			handles.push_back(PluginHandle(const_cast<MiddlewareCore *>(this), plugin_lifetime_, access.Model(),
			                               access.Generation(), stat.name, stat.type));
		}
		return handles;
	});
}

} // namespace mujoco_ros::middleware
