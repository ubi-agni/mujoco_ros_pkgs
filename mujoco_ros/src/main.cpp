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
		ROS_FATAL("No modelfile was provided, simulation cannot start!");
		return -1;
	}

	std::thread sim_thread(MujocoSim::init, filename);


	double control_rate = 0.001;
	ros::Rate loop_rate(1 / control_rate);

	while (ros::ok() && MujocoSim::isUp()) {
		loop_rate.sleep();
	}

	MujocoSim::requestExternalShutdow();
	sim_thread.join();

	ROS_INFO("Franka MuJoCo node is terminating");
}
