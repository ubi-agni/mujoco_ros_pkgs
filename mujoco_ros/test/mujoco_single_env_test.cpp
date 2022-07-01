#include <gtest/gtest.h>

#include <mujoco_ros_msgs/SetPause.h>

#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/common_types.h>

#include <mujoco_ros_msgs/SetPause.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "mujoco_ros_test_node");
	return RUN_ALL_TESTS();
}

namespace unit_testing {
class MujocoRosFixture : public ::testing::Test
{
protected:
	std::shared_ptr<ros::NodeHandle> nh;
	MujocoSim::mjModelPtr getModel(MujocoSim::MujocoEnvPtr env) { return env->model; }
	MujocoSim::mjDataPtr getData(MujocoSim::MujocoEnvPtr env) { return env->data; }

	virtual void SetUp()
	{
		nh.reset(new ros::NodeHandle("~"));
		nh->setParam("unpause", true);
		nh->setParam("visualize", false);
	}

	virtual void TearDown() {}
};

TEST_F(MujocoRosFixture, init_with_model)
{
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	std::thread mjThread(MujocoSim::init, xml_path);

	double time = ros::Time::now().toSec();
	EXPECT_GE(ros::Time::now().toNSec(), 0.0) << "Time should be running!";
	std::this_thread::sleep_for(std::chrono::seconds(1));
	MujocoSim::requestExternalShutdown();
	EXPECT_GE(ros::Time::now().toSec(), 0.001 * 100) << "Time should have kept running";
	mjThread.join();
}

TEST_F(MujocoRosFixture, pause_unpause)
{
	nh->setParam("unpause", false);
	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";
	std::thread mjThread(MujocoSim::init, xml_path);

	double time = ros::Time::now().toSec();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	EXPECT_EQ(ros::Time::now().toSec(), time) << "Time should not be running!";

	mujoco_ros_msgs::SetPause srv;
	srv.request.paused = false;

	MujocoSim::detail::setPauseCB(srv.request, srv.response);

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	EXPECT_GT(ros::Time::now().toSec(), time) << "Time should have been moving forward!";

	srv.request.paused = true;
	MujocoSim::detail::setPauseCB(srv.request, srv.response);

	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	time = ros::Time::now().toSec();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	EXPECT_EQ(ros::Time::now().toSec(), time) << "Time should not have moved forward!";

	MujocoSim::requestExternalShutdown();
	mjThread.join();
}

TEST_F(MujocoRosFixture, num_steps)
{
	nh->setParam("num_steps", 100);
	double time = ros::Time::now().toSec();

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/empty_world.xml";

	std::thread mjThread(MujocoSim::init, xml_path);
	mjThread.join();

	EXPECT_NEAR(ros::Time::now().toSec(), time + 0.001 * 100, 0.0001) << "Time should have stopped after 100 steps";
	nh->deleteParam("num_steps");
}

TEST_F(MujocoRosFixture, default_initial_joint_states)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::thread mjThread(MujocoSim::init, xml_path);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	MujocoSim::MujocoEnvPtr env = MujocoSim::detail::unit_testing::getmjEnv();
	MujocoSim::mjDataPtr d      = getData(env);
	MujocoSim::mjModelPtr m     = getModel(env);

	int id_balljoint, id1, id2, id_free;
	id_balljoint = MujocoSim::jointName2id(m.get(), "balljoint");
	id1          = MujocoSim::jointName2id(m.get(), "joint1");
	id2          = MujocoSim::jointName2id(m.get(), "joint2");
	id_free      = MujocoSim::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint]], 1.0) << "'balljoint' w quat should be 1!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 1], 0.0) << "'balljoint' x quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 2], 0.0) << "'balljoint' y quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 3], 0.0) << "'balljoint' z quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id1]], 0.0) << "'joint1' position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id2]], 0.0) << "'joint2' position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free]], 1.0) << "'ball_freejoint' x position should be 1!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 1], 0.0) << "'ball_freejoint' y position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 2], 0.06) << "'ball_freejoint' z position should be 0.06!";

	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint]], 0.0) << "'balljoint' roll vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 1], 0.0) << "'balljoint' pitch vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 2], 0.0) << "'balljoint' yaw vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id1]], 0.0) << "'joint1' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id2]], 0.0) << "'joint2' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free]], 0.0) << "'ball_freejoint' x vel should be 1!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 1], 0.0) << "'ball_freejoint' y vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 2], 0.0) << "'ball_freejoint' z vel should be 0.06!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 3], 0.0) << "'ball_freejoint' roll vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 4], 0.0) << "'ball_freejoint' pitch vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 5], 0.0) << "'ball_freejoint' yaw vel should be 0!";

	MujocoSim::requestExternalShutdown();
	mjThread.join();
}

TEST_F(MujocoRosFixture, custom_initial_joint_states_on_reset)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	std::thread mjThread(MujocoSim::init, xml_path);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	MujocoSim::MujocoEnvPtr env = MujocoSim::detail::unit_testing::getmjEnv();
	MujocoSim::mjDataPtr d      = getData(env);
	MujocoSim::mjModelPtr m     = getModel(env);

	int id_balljoint, id1, id2, id_free;

	id_balljoint = MujocoSim::jointName2id(m.get(), "balljoint");
	id1          = MujocoSim::jointName2id(m.get(), "joint1");
	id2          = MujocoSim::jointName2id(m.get(), "joint2");
	id_free      = MujocoSim::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint]], 1.0) << "'balljoint' w quat should be 1!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 1], 0.0) << "'balljoint' x quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 2], 0.0) << "'balljoint' y quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 3], 0.0) << "'balljoint' z quat should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id1]], 0.0) << "'joint1' position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id2]], 0.0) << "'joint2' position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free]], 1.0) << "'ball_freejoint' x position should be 1!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 1], 0.0) << "'ball_freejoint' y position should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 2], 0.06) << "'ball_freejoint' z position should be 0.06!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 3], 1.0) << "'ball_freejoint' quat w should be 1!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 4], 0.0) << "'ball_freejoint' quat x should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 5], 0.0) << "'ball_freejoint' quat y should be 0!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 6], 0.0) << "'ball_freejoint' quat z should be 0!";

	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint]], 0.0) << "'balljoint' roll vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 1], 0.0) << "'balljoint' pitch vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 2], 0.0) << "'balljoint' yaw vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id1]], 0.0) << "'joint1' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id2]], 0.0) << "'joint2' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free]], 0.0) << "'ball_freejoint' x vel should be 1!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 1], 0.0) << "'ball_freejoint' y vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 2], 0.0) << "'ball_freejoint' z vel should be 0.06!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 3], 0.0) << "'ball_freejoint' roll vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 4], 0.0) << "'ball_freejoint' pitch vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 5], 0.0) << "'ball_freejoint' yaw vel should be 0!";

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	std_srvs::Empty srv;
	MujocoSim::detail::resetCB(srv.request, srv.response);

	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint]], 0.0) << "'balljoint' w quat should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_balljoint] + 1], 0.707, 9e-4) << "'balljoint' x quat should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 2], 0.0) << "'balljoint' y quat should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_balljoint] + 3], 0.707, 9e-4) << "'balljoint' z quat should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id1]], -1.57) << "'joint1' position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id2]], -0.66) << "'joint2' position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free]], 2.0) << "'ball_freejoint' x position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 1], 1.0) << "'ball_freejoint' y position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 2], 1.06) << "'ball_freejoint' z position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 3], 0.0) << "'ball_freejoint' quat w should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_free] + 4], 0.707, 9e-4) << "'ball_freejoint' quat x should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 5], 0.0) << "'ball_freejoint' quat y should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_free] + 6], 0.707, 9e-4) << "'ball_freejoint' quat z should be changed!";

	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint]], 5.0) << "'balljoint' roll vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 1], 5.0) << "'balljoint' pitch vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 2], 10.0) << "'balljoint' yaw vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id1]], 0.0) << "'joint1' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id2]], 1.05) << "'joint2' velocity should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free]], 1.0) << "'ball_freejoint' x vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 1], 2.0) << "'ball_freejoint' y vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 2], 3.0) << "'ball_freejoint' z vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 3], 10.0) << "'ball_freejoint' roll vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 4], 20.0) << "'ball_freejoint' pitch vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 5], 30.0) << "'ball_freejoint' yaw vel should be changed!";

	MujocoSim::requestExternalShutdown();
	mjThread.join();

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
}

TEST_F(MujocoRosFixture, custom_initial_joint_states)
{
	nh->setParam("unpause", false);

	std::string xml_path = ros::package::getPath("mujoco_ros") + "/test/pendulum_world.xml";
	std::map<std::string, std::string> pos_map, vel_map;

	pos_map.insert({ "balljoint", "0 0.707 0 0.707" });
	pos_map.insert({ "joint1", "-1.57" });
	pos_map.insert({ "joint2", "-0.66" });
	pos_map.insert({ "ball_freejoint", "2.0 1.0 1.06 0.0 0.707 0.0 0.707" });

	vel_map.insert({ "balljoint", "5 5 10" });
	vel_map.insert({ "joint2", "1.05" });
	vel_map.insert({ "ball_freejoint", "1.0 2.0 3.0 10 20 30" });

	nh->setParam("initial_joint_positions/joint_map", pos_map);
	nh->setParam("initial_joint_velocities/joint_map", vel_map);

	std::thread mjThread(MujocoSim::init, xml_path);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	MujocoSim::MujocoEnvPtr env = MujocoSim::detail::unit_testing::getmjEnv();
	MujocoSim::mjDataPtr d      = getData(env);
	MujocoSim::mjModelPtr m     = getModel(env);

	int id_balljoint, id1, id2, id_free;

	id_balljoint = MujocoSim::jointName2id(m.get(), "balljoint");
	id1          = MujocoSim::jointName2id(m.get(), "joint1");
	id2          = MujocoSim::jointName2id(m.get(), "joint2");
	id_free      = MujocoSim::jointName2id(m.get(), "ball_freejoint");

	EXPECT_NE(id_balljoint, -1) << "'balljoint' should be found as joint in model!";
	EXPECT_NE(id1, -1) << "'joint1' should be found as joint in model!";
	EXPECT_NE(id2, -1) << "'joint2' should be found as joint in model!";
	EXPECT_NE(id_free, -1) << "'ball_freejoint' should be found as joint in model!";

	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint]], 0.0) << "'balljoint' w quat should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_balljoint] + 1], 0.707, 9e-4) << "'balljoint' x quat should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_balljoint] + 2], 0.0) << "'balljoint' y quat should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_balljoint] + 3], 0.707, 9e-4) << "'balljoint' z quat should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id1]], -1.57) << "'joint1' position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id2]], -0.66) << "'joint2' position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free]], 2.0) << "'ball_freejoint' x position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 1], 1.0) << "'ball_freejoint' y position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 2], 1.06) << "'ball_freejoint' z position should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 3], 0.0) << "'ball_freejoint' quat w should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_free] + 4], 0.707, 9e-4) << "'ball_freejoint' quat x should be changed!";
	EXPECT_EQ(d->qpos[m->jnt_qposadr[id_free] + 5], 0.0) << "'ball_freejoint' quat y should be changed!";
	EXPECT_NEAR(d->qpos[m->jnt_qposadr[id_free] + 6], 0.707, 9e-4) << "'ball_freejoint' quat z should be changed!";

	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint]], 5.0) << "'balljoint' roll vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 1], 5.0) << "'balljoint' pitch vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_balljoint] + 2], 10.0) << "'balljoint' yaw vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id1]], 0.0) << "'joint1' velocity should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id2]], 1.05) << "'joint2' velocity should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free]], 1.0) << "'ball_freejoint' x vel should be 1!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 1], 2.0) << "'ball_freejoint' y vel should be 0!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 2], 3.0) << "'ball_freejoint' z vel should be 0.06!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 3], 10.0) << "'ball_freejoint' roll vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 4], 20.0) << "'ball_freejoint' pitch vel should be changed!";
	EXPECT_EQ(d->qvel[m->jnt_dofadr[id_free] + 5], 30.0) << "'ball_freejoint' yaw vel should be changed!";

	MujocoSim::requestExternalShutdown();
	mjThread.join();

	nh->deleteParam("initial_joint_positions/joint_map");
	nh->deleteParam("initial_joint_velocities/joint_map");
}

} // namespace unit_testing
