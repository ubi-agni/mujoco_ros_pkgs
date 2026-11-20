#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include <mujoco_ros/simulation_control_state.hpp>

namespace mujoco_ros::middleware {

/**
 * INTERNAL host-side collaboration interface between MujocoEnv and MiddlewareCore.
 *
 * NOT a flavor port: the only flavor outbound ports are IConfigSource and ITimePublisher. ROS adapters
 * must never implement or depend on this interface; they call MiddlewareCore. It exists so the Core can
 * host the Simulation Control API rules without linking MujocoEnv.
 *
 * The Core owns the decision rules (Loading Window, admin_hash, step admission checks, Runtime Options
 * epoch). The host (MujocoEnv) owns the mechanics: physics lock, control state, model and queues.
 * Physics step ownership stays on the host.
 */
class IControlBackend
{
public:
	virtual ~IControlBackend() = default;

	/// Runs func under the (recursive) physics-thread mutex.
	virtual void ControlWithPhysicsLock(const std::function<void()> &func) = 0;
	virtual std::uint64_t ControlAdmissionEpoch() const                    = 0;
	virtual SimulationControlSnapshot ControlSnapshot() const              = 0;
	/// Active model options, or nullptr when no model is loaded. Call under the physics lock.
	virtual mjOption *ControlModelOption()                                   = 0;
	virtual bool ControlHasModel() const                                     = 0;
	virtual void ControlNotifySettingsChanged()                              = 0;
	virtual bool ControlVerifyAdminHash(const std::string &hash)             = 0;
	virtual void ControlSetPaused(bool paused, bool notify_settings_changed) = 0;
	virtual void ControlSetAdminHash(const std::string &hash)                = 0;
	virtual bool ControlOnPhysicsThread() const                              = 0;
	/// Blocks until physics released its lock after a pause; false on timeout.
	virtual bool ControlWaitPhysicsQuiescent() = 0;

	virtual bool ControlAdmitManualSteps(int num_steps, ManualStepToken *token)                       = 0;
	virtual ManualStepSnapshot ControlManualStepSnapshot(ManualStepToken token) const                 = 0;
	virtual ManualStepSnapshot ControlWaitManualStepUpdate(ManualStepToken token, int observed) const = 0;
	virtual bool ControlCancelManualSteps(ManualStepToken token)                                      = 0;
	virtual void ControlAcknowledgeManualStep(ManualStepToken token)                                  = 0;

	virtual void ControlRequestReset()                                 = 0;
	virtual void ControlRequestReload()                                = 0;
	virtual int ControlOperationalStatus()                             = 0;
	virtual std::size_t ControlMaxModelStringLength() const            = 0;
	virtual void ControlQueueModelStringLoad(const std::string &model) = 0;
	/// Copies last load error (if buffer given) and returns whether the model is valid.
	virtual bool ControlLoadResult(char *load_error, int error_sz) = 0;

	virtual void ControlLogDebug(const std::string &message) = 0;
	virtual void ControlLogWarn(const std::string &message)  = 0;
	virtual void ControlLogError(const std::string &message) = 0;
};

} // namespace mujoco_ros::middleware
