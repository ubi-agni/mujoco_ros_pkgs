#pragma once

#include <mujoco_ros/middleware/detail/i_control_backend.hpp>

namespace mujoco_ros {

class MujocoEnv;

namespace detail {

/// MujocoEnv's implementation of the internal host-side IControlBackend (mechanics only; not a flavor port).
class EnvControlBackend : public middleware::IControlBackend
{
public:
	explicit EnvControlBackend(MujocoEnv &env) : env_(env) {}

	void ControlWithPhysicsLock(const std::function<void()> &func) override;
	std::uint64_t ControlAdmissionEpoch() const override;
	SimulationControlSnapshot ControlSnapshot() const override;
	mjOption *ControlModelOption() override;
	bool ControlHasModel() const override;
	void ControlNotifySettingsChanged() override;
	bool ControlVerifyAdminHash(const std::string &hash) override;
	void ControlSetPaused(bool paused, bool notify_settings_changed) override;
	void ControlSetAdminHash(const std::string &hash) override;
	bool ControlOnPhysicsThread() const override;
	bool ControlWaitPhysicsQuiescent() override;

	bool ControlAdmitManualSteps(int num_steps, ManualStepToken *token) override;
	ManualStepSnapshot ControlManualStepSnapshot(ManualStepToken token) const override;
	ManualStepSnapshot ControlWaitManualStepUpdate(ManualStepToken token, int observed) const override;
	bool ControlCancelManualSteps(ManualStepToken token) override;
	void ControlAcknowledgeManualStep(ManualStepToken token) override;

	void ControlRequestReset() override;
	void ControlRequestReload() override;
	int ControlOperationalStatus() override;
	std::size_t ControlMaxModelStringLength() const override;
	void ControlQueueModelStringLoad(const std::string &model) override;
	bool ControlLoadResult(char *load_error, int error_sz) override;

	void ControlLogDebug(const std::string &message) override;
	void ControlLogWarn(const std::string &message) override;
	void ControlLogError(const std::string &message) override;

private:
	MujocoEnv &env_;
};

} // namespace detail
} // namespace mujoco_ros
