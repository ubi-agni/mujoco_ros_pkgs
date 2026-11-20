#include <mujoco_ros/middleware/middleware_core.hpp>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <thread>

namespace mujoco_ros::middleware {

namespace {

class NullConfigSource : public IConfigSource
{
public:
	std::optional<bool> GetBool(const std::string &) const override { return std::nullopt; }
	std::optional<int> GetInt(const std::string &) const override { return std::nullopt; }
	std::optional<double> GetDouble(const std::string &) const override { return std::nullopt; }
	std::optional<std::string> GetString(const std::string &) const override { return std::nullopt; }
	std::optional<std::vector<std::string>> GetStringList(const std::string &) const override { return std::nullopt; }
};

class NullTimePublisher : public ITimePublisher
{
public:
	void PublishSimTime(mjtNum) override {}
};

constexpr const char *kLoadingWindowMessage = "Runtime Options unavailable during Loading Window";

} // namespace

MiddlewareCore::MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher)
    : config_source_(config_source), time_publisher_(time_publisher)
{
}

MiddlewareCore::MiddlewareCore(IConfigSource &config_source, ITimePublisher &time_publisher,
                               IControlBackend &control_backend)
    : config_source_(config_source), time_publisher_(time_publisher), control_backend_(&control_backend)
{
}

MiddlewareCore::MiddlewareCore(IControlBackend &control_backend)
    : owned_config_source_(std::make_unique<NullConfigSource>())
    , owned_time_publisher_(std::make_unique<NullTimePublisher>())
    , config_source_(*owned_config_source_)
    , time_publisher_(*owned_time_publisher_)
    , control_backend_(&control_backend)
{
}

MiddlewareCore::~MiddlewareCore() = default;

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

} // namespace mujoco_ros::middleware
