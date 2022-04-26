#include <mujoco_ros/mujoco_sim.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mujoco");

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::NodeHandle nh = ros::NodeHandle("~");

	bool vis = true;

	std::string filename;

	nh.getParam("modelfile", filename);
	ROS_INFO_STREAM("Using modelfile " << filename);

	if (filename.empty()) {
		ROS_WARN("No modelfile was provided, launching empty simulation!");
	}

	MujocoSim::init(filename);
	ROS_INFO("Franka MuJoCo node is terminating");
}
