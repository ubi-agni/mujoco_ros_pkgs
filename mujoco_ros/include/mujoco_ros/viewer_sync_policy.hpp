#pragma once

namespace mujoco_ros {

bool ShouldSyncViewerEachFrame(bool is_passive, bool auto_sync, int operational_status);
bool ShouldShutdownEnvironmentOnViewerExit(bool is_passive);
bool ShouldProcessPendingViewerExitWithoutSync(bool is_passive, bool auto_sync, int operational_status);

struct PendingViewerExitAction
{
	bool set_exit_request             = false;
	bool request_environment_shutdown = false;
};

PendingViewerExitAction ComputePendingViewerExitAction(bool pending_ui_exit, bool is_passive, int exit_request);

} // namespace mujoco_ros
