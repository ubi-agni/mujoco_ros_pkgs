#include <mujoco_ros/viewer_sync_policy.hpp>

namespace mujoco_ros {

bool ShouldSyncViewerEachFrame(bool is_passive, bool auto_sync, int operational_status)
{
	return operational_status == 0 && (!is_passive || auto_sync);
}

bool ShouldShutdownEnvironmentOnViewerExit(bool is_passive)
{
	return !is_passive;
}

bool ShouldProcessPendingViewerExitWithoutSync(bool is_passive, bool auto_sync, int operational_status)
{
	(void)auto_sync;
	(void)operational_status;
	return is_passive;
}

PendingViewerExitAction ComputePendingViewerExitAction(bool pending_ui_exit, bool is_passive, int exit_request)
{
	PendingViewerExitAction action;
	if (!pending_ui_exit || exit_request != 0) {
		return action;
	}
	action.set_exit_request             = true;
	action.request_environment_shutdown = ShouldShutdownEnvironmentOnViewerExit(is_passive);
	return action;
}

} // namespace mujoco_ros
