#pragma once

namespace mujoco_ros {

struct DrainDeferredPhysicsUiResult
{
	int request_manual_steps = 0;
	bool add_to_history      = false;
	bool apply_model_flags   = false;
};

template <typename Pending>
void ClearDeferredPhysicsUi(Pending &pending)
{
	pending.manual_steps              = 0;
	pending.history_after_manual_step = false;
	pending.apply_model_flags         = false;
}

template <typename Pending>
DrainDeferredPhysicsUiResult DrainDeferredPhysicsUi(Pending &pending, bool running, bool has_model)
{
	DrainDeferredPhysicsUiResult result;
	if (pending.manual_steps > 0) {
		if (!running) {
			result.request_manual_steps = pending.manual_steps;
			result.add_to_history       = pending.history_after_manual_step;
		}
		pending.manual_steps              = 0;
		pending.history_after_manual_step = false;
	}
	if (pending.apply_model_flags) {
		result.apply_model_flags  = has_model;
		pending.apply_model_flags = false;
	}
	return result;
}

} // namespace mujoco_ros
