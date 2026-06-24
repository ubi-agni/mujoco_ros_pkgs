#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unistd.h>

#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros_testing_utils/mujoco_env_fixture.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <mujoco_ros_control/default_robot_hw_sim.hpp>
#include <mujoco_ros_control/mujoco_ros_control_plugin.hpp>
#else
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <mujoco_ros_control/ros_two/mujoco_ros_control.hpp>
#include <mujoco_ros_control/ros_two/mujoco_ros_system.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#endif

TEST(MujocoRosControlTypes, PluginDerivesFromMujocoPlugin)
{
	EXPECT_TRUE((std::is_base_of<mujoco_ros::MujocoPlugin, mujoco_ros::control::MujocoRosControlPlugin>::value));
}

namespace {
int run_all_tests(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);
#if MJR_ROS_VERSION == ROS_1
	if (!ros::isInitialized()) {
		ros::init(argc, argv, "mujoco_ros_control_test",
		          ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	}
	const int ret = RUN_ALL_TESTS();
	ros::shutdown();
	return ret;
#else
	if (!rclcpp::ok()) {
		rclcpp::init(argc, argv);
	}
	const int ret = RUN_ALL_TESTS();
	rclcpp::shutdown();
	return ret;
#endif
}
} // namespace

#if MJR_ROS_VERSION == ROS_1
TEST(MujocoRosControlTypes, DefaultHardwareDerivesFromRobotHWSim)
{
	EXPECT_TRUE((std::is_base_of<mujoco_ros::control::RobotHWSim, mujoco_ros::control::DefaultRobotHWSim>::value));
}

namespace {
std::string write_temp_model(const std::string &xml)
{
	char path[]  = "/tmp/mujoco_ros_control_test_XXXXXX.xml";
	const int fd = mkstemps(path, 4);
	if (fd == -1) {
		throw std::runtime_error("Failed to create temporary model file");
	}
	close(fd);

	std::ofstream file(path);
	file << xml;
	return path;
}

struct ModelData
{
	std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{ nullptr, mj_deleteModel };
	std::unique_ptr<mjData, decltype(&mj_deleteData)> data{ nullptr, mj_deleteData };
};

ModelData load_model(const std::string &xml)
{
	const auto path  = write_temp_model(xml);
	char error[1024] = "";
	ModelData result;
	result.model.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
	unlink(path.c_str());
	if (result.model == nullptr) {
		throw std::runtime_error(error);
	}
	result.data.reset(mj_makeData(result.model.get()));
	return result;
}

void init_ros()
{
	if (!ros::isInitialized()) {
		int argc         = 1;
		char node_name[] = "mujoco_ros_control_test";
		char *argv[]     = { node_name };
		ros::init(argc, argv, "mujoco_ros_control_test", ros::init_options::AnonymousName);
	}
}

std::vector<transmission_interface::TransmissionInfo> transmissions_for(const std::string &interface_name)
{
	transmission_interface::TransmissionInfo transmission;
	transmission.name_ = "hinge_transmission";
	transmission_interface::JointInfo joint;
	joint.name_ = "hinge";
	joint.hardware_interfaces_.push_back("hardware_interface/" + interface_name);
	transmission.joints_.push_back(joint);
	return { transmission };
}

class ExposedDefaultRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
	void SetTestPidGains(double p, double i = 0.0, double d = 0.0)
	{
		pid_p_        = p;
		pid_i_        = i;
		pid_d_        = d;
		use_test_pid_ = true;
	}

	void ForcePidInitFailure() { force_pid_failure_ = true; }

	void SetEffortCommand(double command) { joint_effort_command_[0] = command; }

	void SetPositionCommand(double command) { joint_position_command_[0] = command; }

	void SetVelocityCommand(double command) { joint_velocity_command_[0] = command; }

protected:
	bool InitPidController(unsigned int joint_index, const ros::NodeHandle &pid_nh) override
	{
		if (force_pid_failure_) {
			return false;
		}
		if (!use_test_pid_) {
			return DefaultRobotHWSim::InitPidController(joint_index, pid_nh);
		}
		pid_controllers_[joint_index].initPid(pid_p_, pid_i_, pid_d_, 0.0, 0.0);
		return true;
	}

	void RegisterJointLimits(const std::string & /*joint_name*/,
	                         const hardware_interface::JointHandle & /*joint_handle*/,
	                         const ControlMethod /*ctrl_method*/, const ros::NodeHandle & /*joint_limit_nh*/,
	                         const urdf::Model *const /*urdf_model*/, int *const joint_type, double *const lower_limit,
	                         double *const upper_limit, double *const effort_limit) override
	{
		*joint_type   = urdf::Joint::REVOLUTE;
		*lower_limit  = -1.57;
		*upper_limit  = 1.57;
		*effort_limit = 5.0;
	}

private:
	bool use_test_pid_      = false;
	bool force_pid_failure_ = false;
	double pid_p_           = 0.0;
	double pid_i_           = 0.0;
	double pid_d_           = 0.0;
};

constexpr char kModelWithMotor[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge_act_eff" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
)";

constexpr char kModelWithoutActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)";

constexpr char kModelWithPositionActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_act_pos" joint="hinge" kp="10"/>
  </actuator>
</mujoco>
)";

constexpr char kModelWithVelocityActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_act_vel" joint="hinge" kv="10"/>
  </actuator>
</mujoco>
)";
} // namespace

TEST(DefaultRobotHWSim, EffortCommandUsesMotorWhenPresent)
{
	init_ros();
	auto model_data = load_model(kModelWithMotor);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("EffortJointInterface")));
	hardware.SetEffortCommand(1.25);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_eff");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 1.25, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 0.0, 1e-9);
}

TEST(DefaultRobotHWSim, EffortCommandFallsBackToAppliedForce)
{
	init_ros();
	auto model_data = load_model(kModelWithoutActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("EffortJointInterface")));
	hardware.SetEffortCommand(1.5);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(DefaultRobotHWSim, PositionCommandUsesActuatorTarget)
{
	init_ros();
	auto model_data = load_model(kModelWithPositionActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("PositionJointInterface")));
	hardware.SetPositionCommand(0.7);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_pos");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.7, 1e-9);
}

TEST(DefaultRobotHWSim, VelocityCommandUsesActuatorTarget)
{
	init_ros();
	auto model_data = load_model(kModelWithVelocityActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("VelocityJointInterface")));
	hardware.SetVelocityCommand(0.4);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_vel");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.4, 1e-9);
}

TEST(DefaultRobotHWSim, PositionCommandWithoutActuatorUsesFallbackEffort)
{
	init_ros();
	auto model_data = load_model(kModelWithoutActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.SetTestPidGains(3.0);

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("PositionJointInterface")));
	hardware.ReadSim(ros::Time(1, 0), ros::Duration(0.02));
	hardware.SetPositionCommand(0.5);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(DefaultRobotHWSim, VelocityCommandWithoutActuatorUsesFallbackEffort)
{
	init_ros();
	auto model_data = load_model(kModelWithoutActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.SetTestPidGains(2.0);

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("VelocityJointInterface")));
	hardware.ReadSim(ros::Time(1, 0), ros::Duration(0.02));
	hardware.SetVelocityCommand(0.75);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(DefaultRobotHWSim, PositionFallbackRequiresPidGains)
{
	init_ros();
	auto model_data = load_model(kModelWithoutActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.ForcePidInitFailure();

	EXPECT_FALSE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                              transmissions_for("PositionJointInterface")));
}

TEST(DefaultRobotHWSim, VelocityFallbackRequiresPidGains)
{
	init_ros();
	auto model_data = load_model(kModelWithoutActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.ForcePidInitFailure();

	EXPECT_FALSE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                              transmissions_for("VelocityJointInterface")));
}

TEST(DefaultRobotHWSim, IgnoreActuatorsForcesFallback)
{
	init_ros();
	auto model_data = load_model(kModelWithMotor);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("EffortJointInterface"), true));
	hardware.SetEffortCommand(2.0);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_eff");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 2.0, 1e-9);
}

TEST(DefaultRobotHWSim, IgnoreActuatorsForcesPositionFallback)
{
	init_ros();
	auto model_data = load_model(kModelWithPositionActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.SetTestPidGains(3.0);

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("PositionJointInterface"), true));
	hardware.ReadSim(ros::Time(1, 0), ros::Duration(0.02));
	hardware.SetPositionCommand(0.5);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_pos");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(DefaultRobotHWSim, IgnoreActuatorsForcesVelocityFallback)
{
	init_ros();
	auto model_data = load_model(kModelWithVelocityActuator);
	ExposedDefaultRobotHWSim hardware;
	ros::NodeHandle node_handle;
	hardware.SetTestPidGains(2.0);

	ASSERT_TRUE(hardware.InitSim(model_data.model.get(), model_data.data.get(), nullptr, "", node_handle, nullptr,
	                             transmissions_for("VelocityJointInterface"), true));
	hardware.ReadSim(ros::Time(1, 0), ros::Duration(0.02));
	hardware.SetVelocityCommand(0.75);
	hardware.WriteSim(ros::Time(1, 0), ros::Duration(0.02));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_vel");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}
#else
TEST(MujocoRosControlTypes, DefaultSystemDerivesFromSystemInterface)
{
	EXPECT_TRUE((std::is_base_of<hardware_interface::SystemInterface, mujoco_ros::control::MujocoRosSystem>::value));
}

namespace {
std::string write_temp_model(const std::string &xml)
{
	char path[]  = "/tmp/mujoco_ros_control_test_XXXXXX.xml";
	const int fd = mkstemps(path, 4);
	if (fd == -1) {
		throw std::runtime_error("Failed to create temporary model file");
	}
	close(fd);

	std::ofstream file(path);
	file << xml;
	return path;
}

struct ModelData
{
	std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{ nullptr, mj_deleteModel };
	std::unique_ptr<mjData, decltype(&mj_deleteData)> data{ nullptr, mj_deleteData };
};

ModelData load_model(const std::string &xml)
{
	const auto path  = write_temp_model(xml);
	char error[1024] = "";
	ModelData result;
	result.model.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
	unlink(path.c_str());
	if (result.model == nullptr) {
		throw std::runtime_error(error);
	}
	result.data.reset(mj_makeData(result.model.get()));
	return result;
}

hardware_interface::HardwareInfo hardware_info(const std::string &interface_name, bool ignore_actuators = false,
                                               double gain = 0.0)
{
	hardware_interface::HardwareInfo info;
	info.name                                    = "test_system";
	info.type                                    = "system";
	info.hardware_parameters["ignore_actuators"] = ignore_actuators ? "true" : "false";

	hardware_interface::ComponentInfo joint;
	joint.name = "hinge";
	if (gain > 0.0) {
		joint.parameters[interface_name == "position" ? "kp" : "kv"] = std::to_string(gain);
	}
	joint.parameters["effort_limit"] = "5";

	hardware_interface::InterfaceInfo command;
	command.name = interface_name;
	joint.command_interfaces.push_back(command);

	hardware_interface::InterfaceInfo state;
	state.name = "position";
	joint.state_interfaces.push_back(state);
	state.name = "velocity";
	joint.state_interfaces.push_back(state);
	state.name = "effort";
	joint.state_interfaces.push_back(state);

	info.joints.push_back(joint);
	return info;
}

void init_rclcpp()
{
	if (!rclcpp::ok()) {
		int argc    = 0;
		char **argv = nullptr;
		rclcpp::init(argc, argv);
	}
}

constexpr char kModelWithMotor[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge_act_eff" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
)";

constexpr char kModelWithoutActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)";

constexpr char kModelWithPositionActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_act_pos" joint="hinge" kp="10"/>
  </actuator>
</mujoco>
)";

constexpr char kModelWithVelocityActuator[] = R"(
<mujoco model="control_test">
  <worldbody>
    <body name="body">
      <joint name="hinge" type="hinge"/>
      <geom type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_act_vel" joint="hinge" kv="10"/>
  </actuator>
</mujoco>
)";
} // namespace

TEST(MujocoRosControlSystem, EffortCommandUsesMotorWhenPresent)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithMotor);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(
	    system.initSim(node, hardware_info("effort"), model_data.model.get(), model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(1.25);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_eff");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 1.25, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 0.0, 1e-9);
}

TEST(MujocoRosControlSystem, EffortCommandFallsBackToAppliedForce)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithoutActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_fallback");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(
	    system.initSim(node, hardware_info("effort"), model_data.model.get(), model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(1.5);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(MujocoRosControlSystem, PositionCommandUsesActuatorTarget)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithPositionActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_position");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(
	    system.initSim(node, hardware_info("position"), model_data.model.get(), model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.7);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_pos");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.7, 1e-9);
}

TEST(MujocoRosControlSystem, VelocityCommandUsesActuatorTarget)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithVelocityActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_velocity");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(
	    system.initSim(node, hardware_info("velocity"), model_data.model.get(), model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.4);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_vel");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.4, 1e-9);
}

TEST(MujocoRosControlSystem, PositionCommandWithoutActuatorUsesFallbackEffort)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithoutActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_position_fallback");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(system.initSim(node, hardware_info("position", false, 3.0), model_data.model.get(),
	                           model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.5);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(MujocoRosControlSystem, VelocityCommandWithoutActuatorUsesFallbackEffort)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithoutActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_velocity_fallback");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(system.initSim(node, hardware_info("velocity", false, 2.0), model_data.model.get(),
	                           model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.75);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int joint_id = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(MujocoRosControlSystem, PositionFallbackRequiresGain)
{
	init_rclcpp();
	auto model_data = load_model(kModelWithoutActuator);
	auto node       = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_position_missing_gain");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	EXPECT_FALSE(
	    system.initSim(node, hardware_info("position"), model_data.model.get(), model_data.data.get(), update_rate));
}

TEST(MujocoRosControlSystem, VelocityFallbackRequiresGain)
{
	init_rclcpp();
	auto model_data = load_model(kModelWithoutActuator);
	auto node       = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_velocity_missing_gain");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	EXPECT_FALSE(
	    system.initSim(node, hardware_info("velocity"), model_data.model.get(), model_data.data.get(), update_rate));
}

TEST(MujocoRosControlSystem, IgnoreActuatorsForcesFallback)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithMotor);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_ignore");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(
	    system.initSim(node, hardware_info("effort", true), model_data.model.get(), model_data.data.get(), update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(2.0);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/effort" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_eff");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 2.0, 1e-9);
}

TEST(MujocoRosControlSystem, IgnoreActuatorsForcesPositionFallback)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithPositionActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_ignore_position");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(system.initSim(node, hardware_info("position", true, 3.0), model_data.model.get(), model_data.data.get(),
	                           update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.5);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/position" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_pos");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}

TEST(MujocoRosControlSystem, IgnoreActuatorsForcesVelocityFallback)
{
	init_rclcpp();
	auto model_data          = load_model(kModelWithVelocityActuator);
	auto node                = std::make_shared<rclcpp_lifecycle::LifecycleNode>("control_test_node_ignore_velocity");
	unsigned int update_rate = 50;
	mujoco_ros::control::MujocoRosSystem system;

	ASSERT_TRUE(system.initSim(node, hardware_info("velocity", true, 2.0), model_data.model.get(), model_data.data.get(),
	                           update_rate));
	auto commands = system.export_command_interfaces();
	ASSERT_EQ(commands.size(), 1u);
	commands[0].set_value(0.75);
	ASSERT_EQ(system.prepare_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);
	ASSERT_EQ(system.perform_command_mode_switch({ "hinge/velocity" }, {}), hardware_interface::return_type::OK);

	system.write(rclcpp::Time(1, 0, RCL_STEADY_TIME), rclcpp::Duration(1, 0));

	const int actuator_id = mj_name2id(model_data.model.get(), mjOBJ_ACTUATOR, "hinge_act_vel");
	const int joint_id    = mj_name2id(model_data.model.get(), mjOBJ_JOINT, "hinge");
	EXPECT_NEAR(model_data.data->ctrl[actuator_id], 0.0, 1e-9);
	EXPECT_NEAR(model_data.data->qfrc_applied[model_data.model->jnt_dofadr[joint_id]], 1.5, 1e-9);
}
#endif

int main(int argc, char **argv)
{
	return run_all_tests(argc, argv);
}
