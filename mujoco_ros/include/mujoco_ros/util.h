#include <mujoco_ros/common_types.h>
#include <mujoco/mujoco.h>

namespace mujoco_ros::util {

static inline int jointName2id(mjModel *m, const std::string &joint_name,
                               const std::string &robot_namespace = std::string())
{
	int result = mj_name2id(m, mjOBJ_JOINT, joint_name.c_str());
	if (result == -1 && robot_namespace.size() > 0) {
		ROS_DEBUG_STREAM("Trying to find without namespace (" << joint_name.substr(robot_namespace.size()) << ")");
		result = mj_name2id(m, mjOBJ_JOINT, joint_name.substr(robot_namespace.size()).c_str());
	}
	return result;
}

} // namespace mujoco_ros::util
