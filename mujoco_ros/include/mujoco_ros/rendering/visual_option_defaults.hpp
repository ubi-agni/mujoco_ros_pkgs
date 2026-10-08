#pragma once

#include <mujoco/mujoco.h>

namespace mujoco_ros {

// Shared interactive/offscreen defaults so viewer and offscreen cameras match.
// MuJoCo's mjv_defaultOption enables geom groups 0/1/2; we hide group 2.
inline void ApplyInteractiveViewerGeomDefaults(mjvOption *opt)
{
	if (opt == nullptr) {
		return;
	}
	opt->geomgroup[2] = 0;
}

} // namespace mujoco_ros
