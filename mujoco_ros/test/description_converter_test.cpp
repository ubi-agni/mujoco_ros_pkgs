#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <mujoco_ros/description_converter.hpp>

#include <any>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <sstream>

namespace mju = mujoco_ros;
namespace fs  = std::filesystem;

class DescriptionConverterTest : public ::testing::Test
{
protected:
	static std::string Resource(const std::string &name) { return std::string(TEST_RESOURCES_DIR) + "/" + name; }
};

TEST_F(DescriptionConverterTest, GeneratesNoActuatorsWhenFlagIsFalse)
{
	auto result    = mju::ConvertDescription(Resource("ros2_control_robot.urdf"), "", {}, false);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	EXPECT_EQ(model->nu, 0);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, GeneratesActuatorsWhenFlagIsTrue)
{
	auto result    = mju::ConvertDescription(Resource("ros2_control_robot.urdf"), "", {}, true);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	EXPECT_EQ(model->nu, 5);
	EXPECT_NE(mj_name2id(model, mjOBJ_ACTUATOR, "pos_joint_act_pos"), -1);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, GeneratesTorqueActuatorClampedToUrdfEffortLimit)
{
	auto result    = mju::ConvertDescription(Resource("ros2_control_robot.urdf"), "", {}, true);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, "eff_joint_act_eff");
	ASSERT_NE(actuator, -1);
	EXPECT_EQ(model->actuator_gaintype[actuator], mjGAIN_FIXED);
	EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN * actuator], 1.0);
	EXPECT_EQ(model->actuator_biastype[actuator], mjBIAS_NONE);
	EXPECT_DOUBLE_EQ(model->actuator_gear[6 * actuator], 1.0);
	EXPECT_EQ(model->actuator_forcelimited[actuator], mjLIMITED_TRUE);
	EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator], -10.0);
	EXPECT_DOUBLE_EQ(model->actuator_forcerange[2 * actuator + 1], 10.0);
	EXPECT_EQ(model->actuator_ctrllimited[actuator], mjLIMITED_TRUE)
	    << "Torque actuators must clamp ctrl input to the URDF effort limit, matching position/velocity actuators.";
	EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[2 * actuator], -10.0);
	EXPECT_DOUBLE_EQ(model->actuator_ctrlrange[2 * actuator + 1], 10.0);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, RejectsMissingJointLimitAndMimicFollower)
{
	EXPECT_THROW(mju::ConvertDescription(Resource("ros2_control_missing_joint.urdf"), "", {}, true), std::runtime_error);
	EXPECT_THROW(mju::ConvertDescription(Resource("ros2_control_missing_limit.urdf"), "", {}, true), std::runtime_error);
	EXPECT_THROW(mju::ConvertDescription(Resource("ros2_control_mimic_conflict.urdf"), "", {}, true),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, DeduplicatesRepeatedCommandInterfaces)
{
	const std::string urdf =
	    R"xml(<robot><ros2_control><joint name="joint"><command_interface name="position"/><command_interface name="position"/></joint></ros2_control></robot>)xml";
	auto interfaces = mju::ParseRos2ControlCommandInterfaces(urdf);
	ASSERT_EQ(interfaces.at("joint"), std::vector<std::string>({ "position" }));
}

TEST_F(DescriptionConverterTest, ParsesOneCommandInterfacePerJoint)
{
	std::ifstream in(Resource("ros2_control_robot.urdf"));
	std::ostringstream buf;
	buf << in.rdbuf();

	auto interfaces = mju::ParseRos2ControlCommandInterfaces(buf.str());
	ASSERT_EQ(interfaces.count("pos_joint"), 1u);
	EXPECT_EQ(interfaces.at("pos_joint"), std::vector<std::string>({ "position" }));
	ASSERT_EQ(interfaces.count("vel_joint"), 1u);
	EXPECT_EQ(interfaces.at("vel_joint"), std::vector<std::string>({ "velocity" }));
	ASSERT_EQ(interfaces.count("eff_joint"), 1u);
	EXPECT_EQ(interfaces.at("eff_joint"), std::vector<std::string>({ "effort" }));
}

TEST_F(DescriptionConverterTest, ParsesMultipleCommandInterfacesOnTheSameJoint)
{
	std::ifstream in(Resource("ros2_control_robot.urdf"));
	std::ostringstream buf;
	buf << in.rdbuf();

	auto interfaces = mju::ParseRos2ControlCommandInterfaces(buf.str());
	ASSERT_EQ(interfaces.count("dual_joint"), 1u);
	EXPECT_EQ(interfaces.at("dual_joint"), std::vector<std::string>({ "position", "velocity" }));
}

TEST_F(DescriptionConverterTest, ReturnsEmptyMapWhenNoRos2ControlBlockPresent)
{
	auto interfaces = mju::ParseRos2ControlCommandInterfaces("<robot name=\"r\"><link name=\"l\"/></robot>");
	EXPECT_TRUE(interfaces.empty());
}

TEST_F(DescriptionConverterTest, ThrowsOnCommandInterfaceMissingNameAttribute)
{
	std::ifstream in(Resource("ros2_control_missing_interface_name.urdf"));
	std::ostringstream buf;
	buf << in.rdbuf();

	EXPECT_THROW(mju::ParseRos2ControlCommandInterfaces(buf.str()), std::runtime_error);
}

TEST_F(DescriptionConverterTest, RejectsMissingOrBlankRos2ControlNames)
{
	EXPECT_THROW(mju::ParseRos2ControlCommandInterfaces("<robot><ros2_control><joint><command_interface "
	                                                    "name=\"position\"/></joint></ros2_control></robot>"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseRos2ControlCommandInterfaces("<robot><ros2_control><joint name=\"\"><command_interface "
	                                                    "name=\"position\"/></joint></ros2_control></robot>"),
	             std::runtime_error);
	EXPECT_THROW(
	    mju::ParseRos2ControlCommandInterfaces(
	        R"xml(<robot><ros2_control><joint name=" 	 "><command_interface name="position"/></joint></ros2_control></robot>)xml"),
	    std::runtime_error);
	EXPECT_THROW(mju::ParseRos2ControlCommandInterfaces("<robot><ros2_control><joint name=\"joint\"><command_interface "
	                                                    "name=\"\"/></joint></ros2_control></robot>"),
	             std::runtime_error);
	EXPECT_THROW(
	    mju::ParseRos2ControlCommandInterfaces(
	        R"xml(<robot><ros2_control><joint name="joint"><command_interface name=" 	 "/></joint></ros2_control></robot>)xml"),
	    std::runtime_error);
}

TEST_F(DescriptionConverterTest, ParsesMimicFollowerToDriverMapping)
{
	std::ifstream in(Resource("mimic_robot.urdf"));
	std::ostringstream buf;
	buf << in.rdbuf();

	auto followers = mju::ParseMimicFollowers(buf.str());
	ASSERT_EQ(followers.count("follower_joint"), 1u);
	EXPECT_EQ(followers.at("follower_joint"), "driver_joint");
	EXPECT_EQ(followers.count("driver_joint"), 0u);
}

TEST_F(DescriptionConverterTest, ReturnsEmptyMapWhenNoMimicTagsPresent)
{
	std::ifstream in(Resource("two_link_robot.urdf"));
	std::ostringstream buf;
	buf << in.rdbuf();

	auto followers = mju::ParseMimicFollowers(buf.str());
	EXPECT_TRUE(followers.empty());
}

TEST_F(DescriptionConverterTest, RejectsMissingEmptyOrBlankMimicNames)
{
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint><mimic joint=\"driver\"/></joint></robot>"), std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"\"><mimic joint=\"driver\"/></joint></robot>"), std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers(R"xml(<robot><joint name=" 	 "><mimic joint="driver"/></joint></robot>)xml"), std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic/></joint></robot>"), std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic joint=\"\"/></joint></robot>"), std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers(R"xml(<robot><joint name="follower"><mimic joint=" 	 "/></joint></robot>)xml"), std::runtime_error);
}

TEST_F(DescriptionConverterTest, RejectsMultipleMimicElementsIncludingMalformedSecondElement)
{
	EXPECT_THROW(mju::ParseMimicFollowers(
	                 "<robot><joint name=\"follower\"><mimic joint=\"driver\"/><mimic/></joint></robot>"),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, RejectsDuplicateMimicFollowerDeclarations)
{
	EXPECT_THROW(mju::ParseMimicFollowers(
	                 "<robot><joint name=\"follower\"><mimic joint=\"driver_a\"/></joint>"
	                 "<joint name=\"follower\"><mimic joint=\"driver_b\"/></joint></robot>"),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, ProducesAStandaloneCompilableSpecWithStrippathSet)
{
	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));
	ASSERT_NE(result.spec, nullptr);
	EXPECT_EQ(result.spec->strippath, 1);

	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << "spec did not compile standalone";
	EXPECT_EQ(model->nbody, 3); // world + base_link + link_1

	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, CarriesTheDisableCollisionsExclusion)
{
	auto result    = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));
	mjModel *model = mj_compile(result.spec, nullptr);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(model->nexclude, 1);

	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, ReturnsCanonicalSrdfActuatorOverrideAndUsesItForActuators)
{
	const std::string urdf =
	    R"xml(<robot name="r"><link name="base_link"/><link name="link_1"><inertial><mass value="1"/><inertia ixx=".01" ixy="0" ixz="0" iyy=".01" iyz="0" izz=".01"/></inertial></link><joint name="joint_1" type="revolute"><parent link="base_link"/><child link="link_1"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint><ros2_control name="s" type="system"><joint name="joint_1"><command_interface name="position"/></joint></ros2_control></robot>)xml";
	const auto urdf_path = fs::temp_directory_path() / "mujoco_ros_srdf_flow.urdf";
	const auto srdf_path = fs::temp_directory_path() / "mujoco_ros_srdf_flow.srdf";
	{
		std::ofstream out(urdf_path);
		out << urdf;
	}
	{
		std::ofstream out(srdf_path);
		out << R"xml(<robot name="r"><extended_params name="joint_1"><mujoco_actuator kp="77" armature="2"/></extended_params></robot>)xml";
	}
	auto result = mju::ConvertDescription(urdf_path.string(), srdf_path.string(), {}, true);
	ASSERT_TRUE(result.extended_params.at("joint_1").actuator.has_value());
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, "joint_1_act_pos");
	ASSERT_GE(actuator, 0);
	EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN * actuator], 77.0);
	EXPECT_DOUBLE_EQ(model->dof_armature[mj_name2id(model, mjOBJ_JOINT, "joint_1")], 2.0);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
	fs::remove(urdf_path);
	fs::remove(srdf_path);
}

TEST_F(DescriptionConverterTest, WarnsAndSkipsUnhandledCustomSrdfTagsInConverter)
{
	mju::ConversionResult result{};
	ASSERT_NO_THROW(result = mju::ConvertDescription(Resource("two_link_robot.urdf"),
	                                                 Resource("srdf_extended_params_with_custom_tags.srdf")));
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, LoadsNcfgShapedComplianceTagWithoutThrowing)
{
	// Exact repro from the bug report: .ncfg's space-separated vector
	// <compliance offset="x y z" compliance="c1..c6"/> used to throw
	// "invalid finite-number attribute 'v1'" because core tried (and
	// failed) to parse it as a built-in tag. It must now load silently.
	const fs::path srdf_path = fs::temp_directory_path() / "description_converter_ncfg_compliance.srdf";
	{
		std::ofstream srdf(srdf_path);
		srdf << "<robot name=\"two_link_robot\"><extended_params name=\"joint_1\">"
		     << "<compliance offset=\"0 0 0\" compliance=\"0 0 0 0 0 0\"/>"
		     << "</extended_params></robot>";
	}
	mju::ConversionResult result{};
	ASSERT_NO_THROW(result = mju::ConvertDescription(Resource("two_link_robot.urdf"), srdf_path.string()));
	fs::remove(srdf_path);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, DispatchesToGlobalHandlerWithJointNameTagNameXmlAndUserDataRoundTrip)
{
	struct RecordedDispatch
	{
		std::string joint_name;
		std::string tag_name;
		std::string value_attr;
	};
	RecordedDispatch recorded;

	mju::RegisterExtendedParamsHandler("convert_description_dispatch_test_tag",
	                                   [](const std::string &joint_name, const std::string &tag_name,
	                                      const tinyxml2::XMLElement &xml, mju::ConverterExtensionContext &context) {
		                                   auto *out       = std::any_cast<RecordedDispatch *>(context.user_data);
		                                   out->joint_name = joint_name;
		                                   out->tag_name   = tag_name;
		                                   out->value_attr =
		                                       xml.Attribute("value") != nullptr ? xml.Attribute("value") : "";
	                                   });

	const fs::path srdf_path = fs::temp_directory_path() / "description_converter_dispatch_user_data.srdf";
	{
		std::ofstream srdf(srdf_path);
		srdf << "<robot name=\"two_link_robot\"><extended_params name=\"joint_1\">"
		     << "<convert_description_dispatch_test_tag value=\"42\"/></extended_params></robot>";
	}

	mju::ConverterExtensionContext context;
	context.user_data = &recorded;
	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), srdf_path.string(), {}, false, &context);
	fs::remove(srdf_path);

	EXPECT_EQ(recorded.joint_name, "joint_1");
	EXPECT_EQ(recorded.tag_name, "convert_description_dispatch_test_tag");
	EXPECT_EQ(recorded.value_attr, "42");

	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, AssignsStableBodyDerivedNamesToBlankGeoms)
{
	auto names = [this]() -> std::vector<std::string> {
		auto result    = mju::ConvertDescription(Resource("simple_visual_robot.urdf"), "");
		mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
		if (model == nullptr) {
			ADD_FAILURE() << mjs_getError(result.spec);
			mj_deleteSpec(result.spec);
			return {};
		}
		std::vector<std::string> result_names;
		for (int i = 0; i < model->ngeom; ++i) {
			const char *name = mj_id2name(model, mjOBJ_GEOM, i);
			if (name == nullptr) {
				ADD_FAILURE();
				mj_deleteModel(model);
				mj_deleteSpec(result.spec);
				return {};
			}
			result_names.emplace_back(name);
		}
		mj_deleteModel(model);
		mj_deleteSpec(result.spec);
		return result_names;
	};
	const auto first = names();
	ASSERT_FALSE(first.empty());
	EXPECT_EQ(first, names());
	EXPECT_NE(std::find_if(first.begin(), first.end(),
	                       [](const std::string &name) { return name.find("_collision") != std::string::npos; }),
	          first.end());
}

TEST_F(DescriptionConverterTest, ThrowsOnMissingUrdf)
{
	EXPECT_THROW(mju::ConvertDescription("/nonexistent.urdf", ""), std::runtime_error);
}

TEST_F(DescriptionConverterTest, ThrowsOnMalformedSrdf)
{
	EXPECT_THROW(mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("srdf_malformed_exclusion.srdf")),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, ConvertsNestedMeshesAndGlbVisualSibling)
{
	const std::string urdf = Resource("mesh_robot/mesh_robot.urdf");
	auto result            = mju::ConvertDescription(urdf, "");
	ASSERT_NE(result.spec, nullptr);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	EXPECT_GE(model->nmesh, 2);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, AssignsVisualGroup1AndCollisionGroup2)
{
	auto result = mju::ConvertDescription(Resource("mesh_robot/mesh_robot.urdf"), "");
	ASSERT_NE(result.spec, nullptr);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);

	bool saw_visual    = false;
	bool saw_collision = false;
	for (int i = 0; i < model->ngeom; ++i) {
		if (model->geom_contype[i] == 0 && model->geom_conaffinity[i] == 0) {
			EXPECT_EQ(model->geom_group[i], 1);
			saw_visual = true;
		} else {
			EXPECT_EQ(model->geom_group[i], 2);
			saw_collision = true;
		}
	}
	EXPECT_TRUE(saw_visual);
	EXPECT_TRUE(saw_collision);

	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, CollisionOnlyRobotUsesGroup1)
{
	auto result = mju::ConvertDescription(Resource("collision_only_robot/collision_only_robot.urdf"), "");
	ASSERT_NE(result.spec, nullptr);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	ASSERT_GE(model->ngeom, 1);
	for (int i = 0; i < model->ngeom; ++i) {
		EXPECT_EQ(model->geom_group[i], 1);
	}
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, DoesNotModifySourceUrdfOnDisk)
{
	const std::string path = Resource("two_link_robot.urdf");
	std::ifstream before(path);
	std::ostringstream before_buf;
	before_buf << before.rdbuf();
	std::string original = before_buf.str();

	auto result = mju::ConvertDescription(path, Resource("two_link_robot.srdf"));
	mj_deleteSpec(result.spec);

	std::ifstream after(path);
	std::ostringstream after_buf;
	after_buf << after.rdbuf();
	std::string now = after_buf.str();
	EXPECT_EQ(original, now);
}

TEST_F(DescriptionConverterTest, ComposesRobotIntoWorldAtNamedFrame)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));
	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, result.spec, "r0_"));

	mjModel *model = mj_compile(world, nullptr);
	ASSERT_NE(model, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r0_base_link"), -1);

	mj_deleteModel(model);
	mj_deleteSpec(world);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, ComposesRobotWhoseRootIsNotBaseLink)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto result = mju::ConvertDescription(Resource("simple_visual_robot.urdf"), "");
	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, result.spec, "r0_"));

	mjModel *model = mj_compile(world, nullptr);
	ASSERT_NE(model, nullptr) << mjs_getError(world);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r0_chassis"), -1);
	EXPECT_EQ(mj_name2id(model, mjOBJ_BODY, "r0_base_link"), -1);

	int chassis_id = mj_name2id(model, mjOBJ_BODY, "r0_chassis");
	int mast_id    = mj_name2id(model, mjOBJ_BODY, "r0_mast");
	ASSERT_NE(chassis_id, -1);
	ASSERT_NE(mast_id, -1);

	int chassis_geom_count = 0;
	int mast_geom_count    = 0;
	for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
		if (model->geom_bodyid[geom_id] == chassis_id) {
			++chassis_geom_count;
			EXPECT_EQ(model->geom_type[geom_id], mjGEOM_BOX);
		}
		if (model->geom_bodyid[geom_id] == mast_id) {
			++mast_geom_count;
			EXPECT_EQ(model->geom_type[geom_id], mjGEOM_CYLINDER);
		}
	}
	EXPECT_GE(chassis_geom_count, 1);
	EXPECT_GE(mast_geom_count, 1);

	mj_deleteModel(model);
	mj_deleteSpec(world);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, CanAttachRobotToExplicitWorldBody)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto result = mju::ConvertDescription(Resource("simple_visual_robot.urdf"), "");
	mju::ComposeOptions options;
	options.prefix      = "r0_";
	options.world_frame = "world";
	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, result.spec, options));

	mjModel *model = mj_compile(world, nullptr);
	ASSERT_NE(model, nullptr) << mjs_getError(world);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r0_chassis"), -1);

	mj_deleteModel(model);
	mj_deleteSpec(world);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, TwoRobotsWithDistinctPrefixesComposeCleanly)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto r0 = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));
	auto r1 = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));

	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, r0.spec, "r0_"));
	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, r1.spec, "r1_"));

	mjModel *model = mj_compile(world, nullptr);
	ASSERT_NE(model, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r0_base_link"), -1);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r1_base_link"), -1);

	mj_deleteModel(model);
	mj_deleteSpec(world);
	mj_deleteSpec(r0.spec);
	mj_deleteSpec(r1.spec);
}

TEST(LoadModelFromDescription, FourArgumentOverloadsRemainCallableAndUseEmptyPrefix)
{
	using ShortSaveFn =
	    std::string (*)(const std::string &, const std::string &, mjSpec *, const mujoco_ros::MeshPrepOptions &);
	using ShortLoadFn     = std::pair<mjModel *, mjData *> (*)(const std::string &, const std::string &, mjSpec *,
                                                          const mujoco_ros::MeshPrepOptions &);
	const auto short_save = static_cast<ShortSaveFn>(&mujoco_ros::SaveDescriptionToTempMjb);
	const auto short_load = static_cast<ShortLoadFn>(&mujoco_ros::load_model_from_description);

	const std::string urdf = std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf";
	const std::string srdf = std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf";
	const std::string path = short_save(urdf, srdf, nullptr, {});
	mjModel *saved_model   = mj_loadModel(path.c_str(), nullptr);
	std::remove(path.c_str());
	ASSERT_NE(saved_model, nullptr);
	EXPECT_NE(mj_name2id(saved_model, mjOBJ_BODY, "base_link"), -1);
	EXPECT_EQ(mj_name2id(saved_model, mjOBJ_BODY, "r0_base_link"), -1);
	mj_deleteModel(saved_model);

	auto [model, data] = short_load(urdf, srdf, nullptr, {});
	ASSERT_NE(model, nullptr);
	ASSERT_NE(data, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "base_link"), -1);
	EXPECT_EQ(mj_name2id(model, mjOBJ_BODY, "r0_base_link"), -1);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST(LoadModelFromDescription, GenerateActuatorsTruePersistsGeneratedActuatorsAcrossSaveAndLoad)
{
	const std::string urdf = std::string(TEST_RESOURCES_DIR) + "/ros2_control_robot.urdf";

	const std::string path = mujoco_ros::SaveDescriptionToTempMjb(urdf, "", nullptr, {}, true, "");
	mjModel *saved_model   = mj_loadModel(path.c_str(), nullptr);
	std::remove(path.c_str());
	ASSERT_NE(saved_model, nullptr);
	EXPECT_NE(mj_name2id(saved_model, mjOBJ_ACTUATOR, "pos_joint_act_pos"), -1);
	EXPECT_NE(mj_name2id(saved_model, mjOBJ_ACTUATOR, "vel_joint_act_vel"), -1);
	EXPECT_NE(mj_name2id(saved_model, mjOBJ_ACTUATOR, "eff_joint_act_eff"), -1);
	mj_deleteModel(saved_model);

	auto [model, data] = mujoco_ros::load_model_from_description(urdf, "", nullptr, {}, true, "");
	ASSERT_NE(model, nullptr);
	ASSERT_NE(data, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_ACTUATOR, "pos_joint_act_pos"), -1);
	EXPECT_NE(mj_name2id(model, mjOBJ_ACTUATOR, "vel_joint_act_vel"), -1);
	EXPECT_NE(mj_name2id(model, mjOBJ_ACTUATOR, "eff_joint_act_eff"), -1);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST(LoadModelFromDescription, DefaultAttachPrefixIsEmptyNotR0)
{
	auto [model, data] =
	    mujoco_ros::load_model_from_description(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                            std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf");

	ASSERT_NE(model, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "base_link"), -1)
	    << "default attach_prefix must now be empty, producing an unprefixed body name";
	EXPECT_EQ(mj_name2id(model, mjOBJ_BODY, "r0_base_link"), -1) << "must NOT still default to the old 'r0_' prefix";
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST(LoadModelFromDescription, ExplicitAttachPrefixOverridesTheEmptyDefault)
{
	auto [model, data] = mujoco_ros::load_model_from_description(
	    std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	    std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf",
	    /*world_spec=*/nullptr, /*mesh_options=*/{}, /*generate_actuators=*/false,
	    /*attach_prefix=*/"r1_");

	ASSERT_NE(model, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "r1_base_link"), -1);
	EXPECT_EQ(mj_name2id(model, mjOBJ_BODY, "base_link"), -1);

	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST_F(DescriptionConverterTest, RejectedAttachThrowsAndCallerMustDiscardWorldSpec)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto r0 = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));
	auto r1 = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));

	ASSERT_NO_THROW(mju::ComposeIntoWorld(world, r0.spec, "dup_"));
	// Same prefix again -- must be rejected loudly, never silently mis-resolved.
	EXPECT_THROW(mju::ComposeIntoWorld(world, r1.spec, "dup_"), std::runtime_error);
	// After attachment failure: `world` is now poisoned. The contract is "the
	// caller must discard and rebuild it" -- this test only proves the
	// exception surfaces, it deliberately does NOT keep using `world` after.

	mj_deleteSpec(r0.spec);
	mj_deleteSpec(r1.spec);
	// `world`'s ownership is intentionally abandoned here to avoid the
	// confirmed poisoned-spec segfault risk (attachment failure) -- no mj_deleteSpec(world)
	// call after a rejected attach.
}

TEST_F(DescriptionConverterTest, ComposesRobotAtAnExplicitPositionAndOrientation)
{
	char error[1000] = { 0 };
	mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), Resource("two_link_robot.srdf"));

	const mjtNum pos[3]  = { 1.0, 2.0, 3.0 };
	const mjtNum quat[4] = { 1.0, 0.0, 0.0, 0.0 };
	mjsElement *attached = nullptr;
	ASSERT_NO_THROW(attached = mju::ComposeIntoWorld(world, result.spec, "r0_", pos, quat));
	ASSERT_NE(attached, nullptr);

	mjsBody *attached_body = mjs_asBody(attached);
	ASSERT_NE(attached_body, nullptr);
	EXPECT_DOUBLE_EQ(attached_body->pos[0], 1.0);
	EXPECT_DOUBLE_EQ(attached_body->pos[1], 2.0);
	EXPECT_DOUBLE_EQ(attached_body->pos[2], 3.0);
	EXPECT_DOUBLE_EQ(attached_body->quat[0], 1.0);
	EXPECT_DOUBLE_EQ(attached_body->quat[1], 0.0);
	EXPECT_DOUBLE_EQ(attached_body->quat[2], 0.0);
	EXPECT_DOUBLE_EQ(attached_body->quat[3], 0.0);

	mjModel *model = mj_compile(world, nullptr);
	ASSERT_NE(model, nullptr);
	int body_id = mj_name2id(model, mjOBJ_BODY, "r0_base_link");
	ASSERT_NE(body_id, -1);
	EXPECT_DOUBLE_EQ(model->body_pos[3 * body_id + 0], 1.0);
	EXPECT_DOUBLE_EQ(model->body_pos[3 * body_id + 1], 2.0);
	EXPECT_DOUBLE_EQ(model->body_pos[3 * body_id + 2], 3.0);

	mj_deleteModel(model);
	mj_deleteSpec(world);
	mj_deleteSpec(result.spec);
}

TEST(LoadModelFromDescription, NullWorldSpecComposesIntoDefaultWorld)
{
	auto [model, data] =
	    mujoco_ros::load_model_from_description(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                            std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf");

	ASSERT_NE(model, nullptr);
	ASSERT_NE(data, nullptr);
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "base_link"), -1);
	EXPECT_NE(mj_name2id(model, mjOBJ_GEOM, "ground_plane"), -1);
	EXPECT_GE(model->nlight, 1);

	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST(LoadDefaultWorldSpec, ParsesInstalledDefaultWorld)
{
	mjSpec *world = nullptr;
	ASSERT_NO_THROW(world = mujoco_ros::LoadDefaultWorldSpec());
	ASSERT_NE(world, nullptr);
	EXPECT_NE(mjs_findFrame(world, "spawn_frame"), nullptr);
	mj_deleteSpec(world);
}

TEST(LoadModelFromDescription, ComposesIntoASuppliedWorldSpecWithoutMutatingCallersOriginalPointer)
{
	char error[1000] = { 0 };
	mjSpec *world =
	    mj_parseXML((std::string(TEST_RESOURCES_DIR) + "/minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	auto [model, data] =
	    mujoco_ros::load_model_from_description(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                            std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf", world);

	ASSERT_NE(model, nullptr);
	// Composition actually happened in the returned model (unprefixed: default attach_prefix is now "")...
	EXPECT_NE(mj_name2id(model, mjOBJ_BODY, "base_link"), -1);
	// ...but the caller's original world pointer was never mutated: it was
	// only ever composed into via a copy (SaveDescriptionToTempMjb operates
	// on mj_copySpec(world_spec), never world_spec itself).
	EXPECT_EQ(mjs_findBody(world, "base_link"), nullptr);
	// world is still valid and independently compilable, proving it was
	// left in an undisturbed state.
	mjModel *world_model = mj_compile(world, nullptr);
	EXPECT_NE(world_model, nullptr);
	mj_deleteModel(world_model);

	mj_deleteData(data);
	mj_deleteModel(model);
	mj_deleteSpec(world);
}

TEST(LoadModelFromDescription, ThrowsOnMissingUrdf)
{
	EXPECT_THROW(mujoco_ros::load_model_from_description(std::string(TEST_RESOURCES_DIR) + "/does_not_exist.urdf",
	                                                     std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf"),
	             std::runtime_error);
}

TEST(LoadModelFromDescription, PropagatesCollisionRejectionAsRuntimeErrorWithoutCrashing)
{
	char error[1000] = { 0 };
	mjSpec *world =
	    mj_parseXML((std::string(TEST_RESOURCES_DIR) + "/minimal_world.xml").c_str(), nullptr, error, sizeof(error));
	ASSERT_NE(world, nullptr) << error;

	// Pre-attach a robot under "" -- the exact prefix load_model_from_description now
	// defaults to -- so its own internal composition attempt collides on the
	// same prefix, driven through the public entry point (not ComposeIntoWorld
	// directly) to exercise the
	// catch (...) block in SaveDescriptionToTempMjb.
	//
	// NOTE: this only reaches ComposeIntoWorld's up-front prefix-collision
	// check (description_converter.hpp: thrown BEFORE mjs_attach, world_spec
	// left untouched) -- it does NOT reach the documented post-mutation
	// mjs_attach()==nullptr branch that Finding 1's production fix (never
	// mj_deleteSpec-ing compose_target in this catch block) actually guards
	// against. Per this codebase's own findings, MuJoCo 3.3.5's mjs_attach
	// never returns nullptr for a duplicate-prefix collision -- that's why
	// the up-front check exists -- so that literal branch has no known,
	// practically constructible repro with these test fixtures. The
	// production fix is still correct and covers it unconditionally (it
	// never deletes compose_target on ANY ComposeIntoWorld exception,
	// regardless of which throw site fired); this test only confirms the
	// (already-safe) collision-check path still behaves correctly through
	// the public entry point.
	auto r0 = mujoco_ros::ConvertDescription(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                         std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf");
	ASSERT_NO_THROW(mujoco_ros::ComposeIntoWorld(world, r0.spec, ""));
	mj_deleteSpec(r0.spec);

	EXPECT_THROW(mujoco_ros::load_model_from_description(std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf",
	                                                     std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf",
	                                                     world),
	             std::runtime_error);

	// world (the caller's own, never touched by the internal copy-based
	// composition) remains safe to use and delete.
	mj_deleteSpec(world);
}

TEST_F(DescriptionConverterTest, AppliesGravcompToChildBodyOfNamedJoint)
{
	const fs::path srdf_path = fs::temp_directory_path() / "description_converter_valid_gravcomp.srdf";
	{
		std::ofstream srdf(srdf_path);
		srdf << "<robot name=\"two_link_robot\"><extended_params name=\"joint_1\">"
		     << "<mujoco_gravcomp value=\"1.0\"/></extended_params></robot>";
	}
	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), srdf_path.string());
	fs::remove(srdf_path);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const int child_body = mj_name2id(model, mjOBJ_BODY, "link_1");
	ASSERT_GE(child_body, 0);
	EXPECT_DOUBLE_EQ(model->body_gravcomp[child_body], 1.0);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, ThrowsOnMalformedGravcompValue)
{
	EXPECT_THROW(mju::ConvertDescription(Resource("two_link_robot.urdf"),
	                                     Resource("srdf_extended_params_malformed_gravcomp.srdf")),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, SkipsUnsupportedGravcompTargetWithoutTouchingBodies)
{
	auto result    = mju::ConvertDescription(Resource("two_link_robot.urdf"),
	                                         Resource("srdf_extended_params_unsupported_gravcomp.srdf"));
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const int base_body = mj_name2id(model, mjOBJ_BODY, "base_link");
	ASSERT_GE(base_body, 0);
	EXPECT_DOUBLE_EQ(model->body_gravcomp[base_body], 0.0);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}
