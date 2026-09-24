/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  Copyright (c) 2026, Neura Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor Neura Robotics nor
 *     the names of their contributors may be used to endorse or promote
 *     products derived from this software without specific prior written
 *     permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <mujoco_ros/description_converter.hpp>

#include <any>
#include <chrono>
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

	static int CountFreeJointsOnBody(mjModel *model, const std::string &body_name)
	{
		const int body_id = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
		if (body_id < 0)
			return 0;
		int count = 0;
		for (int joint_id = 0; joint_id < model->njnt; ++joint_id) {
			if (model->jnt_bodyid[joint_id] == body_id && model->jnt_type[joint_id] == mjJNT_FREE)
				++count;
		}
		return count;
	}

	static mjSpec *LoadMinimalWorldSpec()
	{
		char error[1000] = { 0 };
		mjSpec *world    = mj_parseXML(Resource("minimal_world.xml").c_str(), nullptr, error, sizeof(error));
		if (world == nullptr)
			throw std::runtime_error(error);
		return world;
	}

	std::pair<mjModel *, mjData *> LoadWithBaseMode(const std::string &urdf, mju::BaseMode base_mode)
	{
		mjSpec *world = LoadMinimalWorldSpec();
		try {
			auto loaded = mju::load_model_from_description(urdf, "", world, {}, false, "", base_mode);
			mj_deleteSpec(world);
			return loaded;
		} catch (...) {
			mj_deleteSpec(world);
			throw;
		}
	}
};

#ifndef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
TEST_F(DescriptionConverterTest, ComposeOptionsDefaultBaseModeIsFixed)
{
	mju::ComposeOptions options;
	EXPECT_EQ(options.base_mode, mju::BaseMode::kFixed);
}

TEST_F(DescriptionConverterTest, UnanchoredRootWithFreeModeAddsOneFreeJoint)
{
	auto [model, data] = LoadWithBaseMode(Resource("unanchored_root_robot.urdf"), mju::BaseMode::kFree);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(CountFreeJointsOnBody(model, "base_link"), 1);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST_F(DescriptionConverterTest, UnanchoredRootWithFixedModeAddsNoFreeJoint)
{
	auto [model, data] = LoadWithBaseMode(Resource("two_link_robot.urdf"), mju::BaseMode::kFixed);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(CountFreeJointsOnBody(model, "base_link"), 0);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST_F(DescriptionConverterTest, WorldAnchoredRootWithAutoModeAddsNoFreeJoint)
{
	auto [model, data] = LoadWithBaseMode(Resource("world_anchored_robot.urdf"), mju::BaseMode::kAuto);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(CountFreeJointsOnBody(model, "base_link"), 0);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST_F(DescriptionConverterTest, UnanchoredRootWithAutoModeAddsOneFreeJoint)
{
	auto [model, data] = LoadWithBaseMode(Resource("unanchored_root_robot.urdf"), mju::BaseMode::kAuto);
	ASSERT_NE(model, nullptr);
	EXPECT_EQ(CountFreeJointsOnBody(model, "base_link"), 1);
	mj_deleteData(data);
	mj_deleteModel(model);
}

TEST_F(DescriptionConverterTest, AmbiguousMultiRootWithAutoModeFailsConversion)
{
	EXPECT_THROW(LoadWithBaseMode(Resource("ambiguous_multi_root_robot.urdf"), mju::BaseMode::kAuto),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, OverloadsRemainDistinctForContextAndBaseMode)
{
	using ExtensionContextSaveFn =
	    std::string (*)(const std::string &, const std::string &, mjSpec *, const mju::MeshPrepOptions &, bool,
	                    const std::string &, mju::ConverterExtensionContext *);
	using ExtensionContextLoadFn = std::pair<mjModel *, mjData *> (*)(
	    const std::string &, const std::string &, mjSpec *, const mju::MeshPrepOptions &, bool, const std::string &,
	    mju::ConverterExtensionContext *);
	using BaseModeLoadFn = std::pair<mjModel *, mjData *> (*)(const std::string &, const std::string &, mjSpec *,
	                                                          const mju::MeshPrepOptions &, bool, const std::string &,
	                                                          mju::BaseMode, mju::ConverterExtensionContext *);
	using BaseModeSaveFn =
	    std::string (*)(const std::string &, const std::string &, mjSpec *, const mju::MeshPrepOptions &, bool,
	                    const std::string &, mju::BaseMode, mju::ConverterExtensionContext *);

	const auto context_save   = static_cast<ExtensionContextSaveFn>(&mju::SaveDescriptionToTempMjb);
	const auto context_load   = static_cast<ExtensionContextLoadFn>(&mju::load_model_from_description);
	const auto base_mode_load = static_cast<BaseModeLoadFn>(&mju::load_model_from_description);
	const auto base_mode_save = static_cast<BaseModeSaveFn>(&mju::SaveDescriptionToTempMjb);

	ASSERT_NE(reinterpret_cast<void *>(context_load), reinterpret_cast<void *>(base_mode_load));
	ASSERT_NE(reinterpret_cast<void *>(context_save), reinterpret_cast<void *>(base_mode_save));
}

TEST_F(DescriptionConverterTest, ExtensionContextOverloadRoutesThroughSaveAndLoad)
{
	struct RecordedDispatch
	{
		std::string joint_name;
		std::string tag_name;
	};
	RecordedDispatch recorded;

	mju::RegisterExtendedParamsHandler("load_model_extension_context_test_tag",
	                                   [&recorded](const std::string &joint_name, const std::string &tag_name,
	                                               const tinyxml2::XMLElement &, mju::ConverterExtensionContext &) {
		                                   recorded.joint_name = joint_name;
		                                   recorded.tag_name   = tag_name;
	                                   });

	const fs::path srdf_path = fs::temp_directory_path() / "description_converter_extension_context_load.srdf";
	{
		std::ofstream srdf(srdf_path);
		srdf << "<robot name=\"two_link_robot\"><extended_params name=\"joint_1\">"
		     << "<load_model_extension_context_test_tag value=\"1\"/></extended_params></robot>";
	}

	using ExtensionContextLoadFn = std::pair<mjModel *, mjData *> (*)(
	    const std::string &, const std::string &, mjSpec *, const mju::MeshPrepOptions &, bool, const std::string &,
	    mju::ConverterExtensionContext *);
	const auto context_load = static_cast<ExtensionContextLoadFn>(&mju::load_model_from_description);

	mju::ConverterExtensionContext context;
	mjSpec *world = LoadMinimalWorldSpec();
	auto [model, data] =
	    context_load(Resource("two_link_robot.urdf"), srdf_path.string(), world, {}, false, "", &context);
	fs::remove(srdf_path);

	ASSERT_NE(model, nullptr);
	EXPECT_EQ(recorded.joint_name, "joint_1");
	EXPECT_EQ(recorded.tag_name, "load_model_extension_context_test_tag");
	EXPECT_EQ(CountFreeJointsOnBody(model, "base_link"), 0) << "legacy overload must keep fixed default base mode";

	mj_deleteData(data);
	mj_deleteModel(model);
	mj_deleteSpec(world);
}

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
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint><mimic joint=\"driver\"/></joint></robot>"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"\"><mimic joint=\"driver\"/></joint></robot>"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers(R"xml(<robot><joint name=" 	 "><mimic joint="driver"/></joint></robot>)xml"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic/></joint></robot>"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic joint=\"\"/></joint></robot>"),
	             std::runtime_error);
	EXPECT_THROW(mju::ParseMimicFollowers(R"xml(<robot><joint name="follower"><mimic joint=" 	 "/></joint></robot>)xml"),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, RejectsMultipleMimicElementsIncludingMalformedSecondElement)
{
	EXPECT_THROW(
	    mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic joint=\"driver\"/><mimic/></joint></robot>"),
	    std::runtime_error);
}

TEST_F(DescriptionConverterTest, RejectsDuplicateMimicFollowerDeclarations)
{
	EXPECT_THROW(mju::ParseMimicFollowers("<robot><joint name=\"follower\"><mimic joint=\"driver_a\"/></joint>"
	                                      "<joint name=\"follower\"><mimic joint=\"driver_b\"/></joint></robot>"),
	             std::runtime_error);
}

TEST_F(DescriptionConverterTest, MimicFollowerGetsAnEqualityConstraintCouplingItToItsDriver)
{
	// mimic_robot.urdf: follower_joint <mimic joint="driver_joint" multiplier="1.0" offset="0.0"/>
	// with generate_actuators=false (a mimic follower normally gets no actuator of its own --
	// this constraint is the *only* thing that stops it from free-swinging).
	auto result    = mju::ConvertDescription(Resource("mimic_robot.urdf"), "", {}, false);
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);

	ASSERT_EQ(model->neq, 1);
	EXPECT_EQ(model->eq_type[0], mjEQ_JOINT);
	const int follower_id = mj_name2id(model, mjOBJ_JOINT, "follower_joint");
	const int driver_id   = mj_name2id(model, mjOBJ_JOINT, "driver_joint");
	ASSERT_NE(follower_id, -1);
	ASSERT_NE(driver_id, -1);
	EXPECT_EQ(model->eq_obj1id[0], follower_id);
	EXPECT_EQ(model->eq_obj2id[0], driver_id);
	EXPECT_DOUBLE_EQ(model->eq_data[mjNEQDATA * 0 + 0], 0.0); // offset
	EXPECT_DOUBLE_EQ(model->eq_data[mjNEQDATA * 0 + 1], 1.0); // multiplier
	EXPECT_TRUE(model->eq_active0[0]);
	// Deliberately using the engine's default solref/solimp -- tightening it
	// destabilizes tiny-inertia followers (see comment in
	// ApplyMimicEqualityConstraints). Instead, the follower joint's armature
	// is raised to condition the constraint numerically.
	EXPECT_DOUBLE_EQ(model->eq_solref[mjNREF * 0 + 0], 0.02);
	EXPECT_DOUBLE_EQ(model->eq_solref[mjNREF * 0 + 1], 1.0);
	const int follower_dof = mj_name2id(model, mjOBJ_JOINT, "follower_joint");
	ASSERT_NE(follower_dof, -1);
	EXPECT_GE(model->jnt_dofadr[follower_dof], 0);
	EXPECT_DOUBLE_EQ(model->dof_armature[model->jnt_dofadr[follower_dof]], 5e-4);

	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
}

TEST_F(DescriptionConverterTest, ApplyMimicEqualityConstraintsThrowsWhenDriverJointIsMissing)
{
	auto result = mju::ConvertDescription(Resource("two_link_robot.urdf"), "", {}, false);
	EXPECT_THROW(mju::ApplyMimicEqualityConstraints(
	                 result.spec, "<robot><joint name=\"joint_1\"><mimic joint=\"nonexistent_joint\"/></joint></robot>"),
	             std::runtime_error);
	mj_deleteSpec(result.spec);
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

namespace {

const char *kWheelRobotUrdf =
    R"xml(<robot name="w"><link name="base_link"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link><link name="wheel_link"><inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial><collision><geometry><cylinder radius="0.05" length="0.02"/></geometry></collision></link><joint name="wheel_joint" type="continuous"><parent link="base_link"/><child link="wheel_link"/><origin xyz="0 0.15 0" rpy="0 1.5708 0"/><axis xyz="0 0 1"/><limit effort="10" velocity="10"/>%FRICTION%</joint></robot>)xml";

struct GeomFriction
{
	bool found;
	double slide;
	double spin;
	double roll;
};

GeomFriction FirstGeomFrictionOfBody(mjModel *model, const char *body_name)
{
	const int body_id = mj_name2id(model, mjOBJ_BODY, body_name);
	GeomFriction out{ false, -1.0, -1.0, -1.0 };
	if (body_id < 0)
		return out;
	for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
		if (model->geom_bodyid[geom_id] != body_id)
			continue;
		out = { true, model->geom_friction[3 * geom_id], model->geom_friction[3 * geom_id + 1],
			     model->geom_friction[3 * geom_id + 2] };
		break;
	}
	return out;
}

} // namespace

TEST_F(DescriptionConverterTest, AppliesUrdfJointFrictionDsToChildLinkGeoms)
{
	std::string urdf = kWheelRobotUrdf;
	urdf.replace(urdf.find("%FRICTION%"), std::string("%FRICTION%").size(),
	             R"xml(<friction ds="0.8" vd="0" k1="0.0001" k2="0.00001" k3="0"/>)xml");
	const auto urdf_path = fs::temp_directory_path() / "mujoco_ros_urdf_friction_ds.urdf";
	{
		std::ofstream out(urdf_path);
		out << urdf;
	}
	auto result    = mju::ConvertDescription(urdf_path.string(), "");
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const GeomFriction wheel = FirstGeomFrictionOfBody(model, "wheel_link");
	EXPECT_TRUE(wheel.found);
	EXPECT_NEAR(wheel.slide, 0.8, 1e-12);
	EXPECT_NEAR(wheel.spin, 0.8 * 0.005, 1e-15);
	EXPECT_NEAR(wheel.roll, 0.8 * 0.0001, 1e-17);
	const GeomFriction base = FirstGeomFrictionOfBody(model, "base_link");
	EXPECT_TRUE(base.found);
	EXPECT_NEAR(base.slide, 1.0, 1e-12);
	EXPECT_NEAR(base.spin, 0.005, 1e-12);
	EXPECT_NEAR(base.roll, 0.0001, 1e-14);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
	fs::remove(urdf_path);
}

TEST_F(DescriptionConverterTest, LeavesGeomFrictionAtMuJoCoDefaultWithoutFrictionTag)
{
	std::string urdf = kWheelRobotUrdf;
	urdf.erase(urdf.find("%FRICTION%"), std::string("%FRICTION%").size());
	const auto urdf_path = fs::temp_directory_path() / "mujoco_ros_urdf_friction_absent.urdf";
	{
		std::ofstream out(urdf_path);
		out << urdf;
	}
	auto result    = mju::ConvertDescription(urdf_path.string(), "");
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const GeomFriction wheel = FirstGeomFrictionOfBody(model, "wheel_link");
	EXPECT_TRUE(wheel.found);
	EXPECT_NEAR(wheel.slide, 1.0, 1e-12);
	EXPECT_NEAR(wheel.spin, 0.005, 1e-12);
	EXPECT_NEAR(wheel.roll, 0.0001, 1e-14);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
	fs::remove(urdf_path);
}

TEST_F(DescriptionConverterTest, SkipsJointFrictionWithoutDsAttribute)
{
	std::string urdf = kWheelRobotUrdf;
	urdf.replace(urdf.find("%FRICTION%"), std::string("%FRICTION%").size(), R"xml(<friction vd="0" k1="0.0001"/>)xml");
	const auto urdf_path = fs::temp_directory_path() / "mujoco_ros_urdf_friction_nods.urdf";
	{
		std::ofstream out(urdf_path);
		out << urdf;
	}
	auto result    = mju::ConvertDescription(urdf_path.string(), "");
	mjModel *model = mju::CompileWithMeshVfs(result.spec, result.basename_to_dir);
	ASSERT_NE(model, nullptr) << mjs_getError(result.spec);
	const GeomFriction wheel = FirstGeomFrictionOfBody(model, "wheel_link");
	EXPECT_TRUE(wheel.found);
	EXPECT_NEAR(wheel.slide, 1.0, 1e-12);
	mj_deleteModel(model);
	mj_deleteSpec(result.spec);
	fs::remove(urdf_path);
}

TEST_F(DescriptionConverterTest, ParseUrdfJointFrictionReturnsDsPerJointAndSkipsMissingDs)
{
	const std::string urdf =
	    R"xml(<robot name="f"><link name="a"/><link name="b"/><link name="c"/><joint name="j1" type="revolute"><parent link="a"/><child link="b"/><friction ds="0.7"/></joint><joint name="j2" type="revolute"><parent link="b"/><child link="c"/></joint><joint name="j3" type="revolute"><parent link="a"/><child link="b"/><friction vd="0" k1="0.0001"/></joint></robot>)xml";
	const auto frictions = mju::ParseUrdfJointFriction(urdf);
	EXPECT_EQ(frictions.size(), 1u);
	EXPECT_NEAR(frictions.at("j1"), 0.7, 1e-12);
}

TEST_F(DescriptionConverterTest, ParseUrdfJointFrictionReturnsEmptyMapWithoutFrictionTags)
{
	std::string urdf = kWheelRobotUrdf;
	urdf.erase(urdf.find("%FRICTION%"), std::string("%FRICTION%").size());
	EXPECT_TRUE(mju::ParseUrdfJointFriction(urdf).empty());
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
#endif

#ifdef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
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

TEST(ModelCache, RepeatedLoadWithUnchangedInputsProducesEquivalentModel)
{
	const std::string urdf = std::string(TEST_RESOURCES_DIR) + "/two_link_robot.urdf";
	const std::string srdf = std::string(TEST_RESOURCES_DIR) + "/two_link_robot.srdf";

	auto [model1, data1] = mujoco_ros::load_model_from_description(urdf, srdf);
	ASSERT_NE(model1, nullptr);
	const int nbody1 = model1->nbody;
	mj_deleteData(data1);
	mj_deleteModel(model1);

	auto [model2, data2] = mujoco_ros::load_model_from_description(urdf, srdf);
	ASSERT_NE(model2, nullptr);
	EXPECT_EQ(model2->nbody, nbody1);
	mj_deleteData(data2);
	mj_deleteModel(model2);
}

TEST(ModelCache, ChangedUrdfContentAtSamePathProducesUpdatedModel)
{
	auto dir = fs::temp_directory_path() / "model_cache_invalidation_test";
	fs::create_directories(dir);
	auto urdf_path = dir / "robot.urdf";

	const std::string two_link = R"(<robot name="r">
  <link name="base_link"/>
  <link name="link_1">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
</robot>)";
	std::ofstream(urdf_path) << two_link;

	auto [model1, data1] = mujoco_ros::load_model_from_description(urdf_path.string(), "");
	ASSERT_NE(model1, nullptr);
	EXPECT_EQ(mj_name2id(model1, mjOBJ_BODY, "link_2"), -1);
	mj_deleteData(data1);
	mj_deleteModel(model1);

	const std::string three_link = R"(<robot name="r">
  <link name="base_link"/>
  <link name="link_1">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="link_2">
    <inertial><mass value="1.0"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/><child link="link_1"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/><child link="link_2"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1"/>
  </joint>
</robot>)";
	std::ofstream(urdf_path) << three_link;

	auto [model2, data2] = mujoco_ros::load_model_from_description(urdf_path.string(), "");
	ASSERT_NE(model2, nullptr);
	EXPECT_NE(mj_name2id(model2, mjOBJ_BODY, "link_2"), -1)
	    << "URDF content changed at the same path -- the cache must not have served the stale first model";
	mj_deleteData(data2);
	mj_deleteModel(model2);
}
#endif

#ifndef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
TEST_F(DescriptionConverterTest, CustomWorldSpecRepeatedLoadStillSucceedsWithoutCaching)
{
	// world_spec != nullptr carries state a content hash can't see -- the cache
	// must stay disabled for this path (fall back to always recompiling) rather
	// than risk serving a wrong cached model.
	auto [model1, data1] = LoadWithBaseMode(Resource("two_link_robot.urdf"), mju::BaseMode::kFixed);
	ASSERT_NE(model1, nullptr);
	const int nbody1 = model1->nbody;
	mj_deleteData(data1);
	mj_deleteModel(model1);

	auto [model2, data2] = LoadWithBaseMode(Resource("two_link_robot.urdf"), mju::BaseMode::kFixed);
	ASSERT_NE(model2, nullptr);
	EXPECT_EQ(model2->nbody, nbody1);
	mj_deleteData(data2);
	mj_deleteModel(model2);
}

namespace {
std::string BuildChainUrdf(const std::string &name, int link_count, double effort)
{
	std::ostringstream out;
	out << "<robot name=\"" << name << "\">\n  <link name=\"link_0\"/>\n";
	for (int i = 1; i <= link_count; ++i) {
		out << "  <link name=\"link_" << i << "\">\n"
		    << "    <inertial><mass value=\"1.0\"/><inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" "
		       "iyz=\"0\" izz=\"0.01\"/></inertial>\n"
		    << "    <visual><geometry><box size=\"0.05 0.05 0.05\"/></geometry></visual>\n"
		    << "    <collision><geometry><box size=\"0.05 0.05 0.05\"/></geometry></collision>\n"
		    << "  </link>\n"
		    << "  <joint name=\"joint_" << i << "\" type=\"revolute\">\n"
		    << "    <parent link=\"link_" << (i - 1) << "\"/><child link=\"link_" << i << "\"/>\n"
		    << "    <axis xyz=\"0 0 1\"/>\n"
		    << "    <limit lower=\"-1.0\" upper=\"1.0\" effort=\"" << effort << "\" velocity=\"1\"/>\n"
		    << "  </joint>\n";
	}
	out << "</robot>\n";
	return out.str();
}
} // namespace

TEST(ModelCache, RepeatedLoadIsMeaningfullyFasterThanARealMiss)
{
	// The persistent model cache survives across test *runs*, not just within
	// one -- a fixed robot name would replay a previous run's cache entry and
	// turn "miss" into another hit. Salt the name so every run's content (and
	// therefore cache key) is genuinely new.
	const auto salt = std::to_string(std::chrono::system_clock::now().time_since_epoch().count());

	auto dir = fs::temp_directory_path() / "model_cache_timing_test";
	fs::create_directories(dir);
	const auto path_a = dir / "chain_a.urdf";
	const auto path_b = dir / "chain_b.urdf";
	std::ofstream(path_a) << BuildChainUrdf("chain_a_" + salt, 400, 10.0);
	std::ofstream(path_b) << BuildChainUrdf("chain_b_" + salt, 400, 11.0); // different content -> different cache key

	// First call pays both one-time process warm-up and the real compile cost;
	// its own timing is not measured (see mesh-index experiment notes: a
	// process's first mj_compile() call is markedly slower than later ones,
	// independent of any cache).
	{
		auto [model, data] = mujoco_ros::load_model_from_description(path_a.string(), "");
		ASSERT_NE(model, nullptr);
		mj_deleteData(data);
		mj_deleteModel(model);
	}

	// A genuine miss, measured after warm-up so it isolates the real per-load cost.
	const auto t0                = std::chrono::steady_clock::now();
	auto [miss_model, miss_data] = mujoco_ros::load_model_from_description(path_b.string(), "");
	const auto t1                = std::chrono::steady_clock::now();
	ASSERT_NE(miss_model, nullptr);
	mj_deleteData(miss_data);
	mj_deleteModel(miss_model);

	// A repeat of the first URDF, unchanged -- must hit the cache.
	const auto t2              = std::chrono::steady_clock::now();
	auto [hit_model, hit_data] = mujoco_ros::load_model_from_description(path_a.string(), "");
	const auto t3              = std::chrono::steady_clock::now();
	ASSERT_NE(hit_model, nullptr);
	mj_deleteData(hit_data);
	mj_deleteModel(hit_model);

	const auto miss_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
	const auto hit_ms  = std::chrono::duration<double, std::milli>(t3 - t2).count();
	EXPECT_LT(hit_ms, miss_ms * 0.5) << "cache hit (" << hit_ms << " ms) should be well under half of a real miss ("
	                                 << miss_ms << " ms)";
}
#endif

#ifndef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
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
#endif

#ifdef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
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
#endif

#ifndef DESCRIPTION_CONVERTER_INTEGRATION_TESTS
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
#endif
