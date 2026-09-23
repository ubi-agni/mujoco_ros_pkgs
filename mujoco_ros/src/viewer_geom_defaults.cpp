#include <mujoco_ros/viewer.hpp>

namespace mujoco_ros {

void ApplyInteractiveViewerGeomDefaults(mjvOption *opt)
{
	if (opt == nullptr) {
		return;
	}
	opt->geomgroup[2] = 0;
}

} // namespace mujoco_ros
