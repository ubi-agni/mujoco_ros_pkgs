#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <py_binding_tools/ros_msg_typecasters.h>

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>
#include <mujoco_ros_msgs/MocapState.h>
#include <geometry_msgs/PoseStamped.h>

#include <mujoco_ros_mocap/mocap_plugin.h>

namespace py = pybind11;

namespace mujoco_ros::mocap {

class MocapPluginAccessor
{
public:
	static mujoco_ros_msgs::MocapState getLastMocapState(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		return plugin.last_mocap_state_;
	}

	static mujoco_ros_msgs::MocapState *getLastMocapStatePtr(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		return &plugin.last_mocap_state_;
	}

	static void setLastMocapState(mujoco_ros::mocap::MocapPlugin &plugin, const mujoco_ros_msgs::MocapState &state)
	{
		if (ValidateMocapMsg(state, plugin.m_)) {
			plugin.last_mocap_state_ = state;
		} else {
			ROS_ERROR("Invalid MocapState provided");
		}
	}

	static mujoco_ros_msgs::MocapState getCurrentMocapsAsMsg(mujoco_ros::mocap::MocapPlugin &plugin)
	{
		mujoco_ros_msgs::MocapState mocap_state;
		for (int i = 0; i < plugin.m_->nbody; ++i) {
			if (plugin.m_->body_mocapid[i] != -1) {
				geometry_msgs::PoseStamped pose_stamped;
				pose_stamped.header.frame_id    = "world"; // Assuming all mocap bodies are in the world frame
				pose_stamped.pose.position.x    = plugin.d_->xpos[3 * i];
				pose_stamped.pose.position.y    = plugin.d_->xpos[3 * i + 1];
				pose_stamped.pose.position.z    = plugin.d_->xpos[3 * i + 2];
				pose_stamped.pose.orientation.w = plugin.d_->xquat[4 * i];
				pose_stamped.pose.orientation.x = plugin.d_->xquat[4 * i + 1];
				pose_stamped.pose.orientation.y = plugin.d_->xquat[4 * i + 2];
				pose_stamped.pose.orientation.z = plugin.d_->xquat[4 * i + 3];
				mocap_state.name.push_back(mj_id2name(plugin.m_, mjOBJ_BODY, i));
				mocap_state.pose.push_back(pose_stamped);
			}
		}
		return mocap_state;
	}
};

} // namespace mujoco_ros::mocap

namespace mujoco_ros::python::mocap {
PYBIND11_MODULE(pymujoco_ros_mocap, m)
{
	py::module::import("mujoco_ros"); // Import mujoco_ros to ensure MujocoPlugin is registered

	py::class_<mujoco_ros::mocap::MocapPlugin, mujoco_ros::MujocoPlugin,
	           std::shared_ptr<mujoco_ros::mocap::MocapPlugin>>(m, "MujocoRosMocapPlugin")
	    .def(py::init<>())
	    .def("get_current_mocaps_as_msg", &mujoco_ros::mocap::MocapPluginAccessor::getCurrentMocapsAsMsg)
	    .def_property("mocap_state", &mujoco_ros::mocap::MocapPluginAccessor::getLastMocapStatePtr,
	                  &mujoco_ros::mocap::MocapPluginAccessor::setLastMocapState,
	                  py::return_value_policy::reference_internal);
}

} // namespace mujoco_ros::python::mocap
