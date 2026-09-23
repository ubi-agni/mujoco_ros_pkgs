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

#include <array>

#include <gtest/gtest.h>
#include <mujoco_ros/description_bundle.hpp>

namespace mju = mujoco_ros;

TEST(DescriptionBundle, ParsesFileSourcedUrdfAndSrdf)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
		{ "srdf.source", "file" },
		{ "srdf.path", "/tmp/robot.srdf" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_EQ(bundle.urdf.kind, mju::DescriptionSource::Kind::kFile);
	EXPECT_EQ(bundle.urdf.path, "/tmp/robot.urdf");
	ASSERT_TRUE(bundle.srdf.has_value());
	EXPECT_EQ(bundle.srdf->kind, mju::DescriptionSource::Kind::kFile);
}

TEST(DescriptionBundle, ParsesTopicSourcedUrdfAndSrdfWithDefaults)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "topic" }, // urdf.topic omitted -- exercises the "robot_description" default
		{ "srdf.source", "topic" },
		{ "srdf.topic", "/custom/robot_description_semantic" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_EQ(bundle.urdf.kind, mju::DescriptionSource::Kind::kTopic);
	EXPECT_EQ(bundle.urdf.topic, "robot_description");
	ASSERT_TRUE(bundle.srdf.has_value());
	EXPECT_EQ(bundle.srdf->kind, mju::DescriptionSource::Kind::kTopic);
	EXPECT_EQ(bundle.srdf->topic, "/custom/robot_description_semantic");
}

TEST(DescriptionBundle, ThrowsWhenSourceIsParam)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "param" }, // "param" is no longer a recognized source kind
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, ReturnsNulloptWhenNoBundleParamsPresent)
{
	std::map<std::string, std::string> flat; // e.g. a plain modelfile-only launch
	EXPECT_FALSE(mju::TryParseDescriptionBundleFromMap(flat).has_value());
}

TEST(DescriptionBundle, ThrowsWhenUrdfSourceIsFileWithoutPath)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" }, // missing urdf.path
		{ "srdf.source", "file" },
		{ "srdf.path", "/tmp/x" },
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, ThrowsOnInvalidSourceKind)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "bogus" },
		{ "srdf.source", "file" },
		{ "srdf.path", "/tmp/x" },
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, ParsesUrdfOnlyWithoutSrdf)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_EQ(bundle.urdf.path, "/tmp/robot.urdf");
	EXPECT_FALSE(bundle.srdf.has_value());
}

TEST(DescriptionBundle, ThrowsWhenOnlySrdfSourcePresent)
{
	std::map<std::string, std::string> flat = {
		{ "srdf.source", "file" },
		{ "srdf.path", "/tmp/robot.srdf" },
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
	EXPECT_THROW(mju::TryParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, ConvertAsciiStlDefaultsFalseWhenAbsent)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_FALSE(bundle.convert_ascii_stl);
}

TEST(DescriptionBundle, ParsesConvertAsciiStlTrueAndFalse)
{
	{
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.convert_ascii_stl", "true" },
		};
		EXPECT_TRUE(mju::ParseDescriptionBundleFromMap(flat).convert_ascii_stl);
	}
	{
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.convert_ascii_stl", "false" },
		};
		EXPECT_FALSE(mju::ParseDescriptionBundleFromMap(flat).convert_ascii_stl);
	}
}

TEST(DescriptionBundle, ThrowsOnInvalidConvertAsciiStlValue)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
		{ "description.convert_ascii_stl", "yes" },
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, GenerateActuatorsDefaultsFalseWhenAbsent)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_FALSE(bundle.generate_actuators);
}

TEST(DescriptionBundle, ParsesGenerateActuatorsTrueAndFalse)
{
	{
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.generate_actuators", "true" },
		};
		EXPECT_TRUE(mju::ParseDescriptionBundleFromMap(flat).generate_actuators);
	}
	{
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.generate_actuators", "false" },
		};
		EXPECT_FALSE(mju::ParseDescriptionBundleFromMap(flat).generate_actuators);
	}
}

TEST(DescriptionBundle, ThrowsOnInvalidGenerateActuatorsValue)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
		{ "description.generate_actuators", "yes" },
	};

	EXPECT_THROW(mju::ParseDescriptionBundleFromMap(flat), std::runtime_error);
}

TEST(DescriptionBundle, RejectsPresentBooleanValuesOutsideExactLowercaseLiterals)
{
	const std::array<const char *, 5> invalid_values = { "", " true", "true ", "TRUE", "False" };
	for (const char *key : { "description.convert_ascii_stl", "description.generate_actuators" }) {
		for (const char *value : invalid_values) {
			std::map<std::string, std::string> flat = {
				{ "urdf.source", "file" },
				{ "urdf.path", "/tmp/robot.urdf" },
				{ key, value },
			};

			try {
				(void)mju::ParseDescriptionBundleFromMap(flat);
				FAIL() << key << " must reject present value '" << value << "'";
			} catch (const std::runtime_error &error) {
				EXPECT_NE(std::string(error.what()).find(key), std::string::npos);
				EXPECT_NE(std::string(error.what()).find("must be 'true' or 'false'"), std::string::npos);
			}
		}
	}
}

TEST(DescriptionBundle, AttachPrefixDefaultsEmptyWhenAbsent)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
	};

	auto bundle = mju::ParseDescriptionBundleFromMap(flat);
	EXPECT_TRUE(bundle.attach_prefix.empty());
}

TEST(DescriptionBundle, ParsesExplicitAttachPrefix)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
		{ "description.attach_prefix", "r1_" },
	};

	EXPECT_EQ(mju::ParseDescriptionBundleFromMap(flat).attach_prefix, "r1_");
}

TEST(DescriptionBundle, BaseModeDefaultsAutoWhenAbsent)
{
	std::map<std::string, std::string> flat = {
		{ "urdf.source", "file" },
		{ "urdf.path", "/tmp/robot.urdf" },
	};

	EXPECT_EQ(mju::ParseDescriptionBundleFromMap(flat).base_mode, mju::BaseMode::kAuto);
}

TEST(DescriptionBundle, ParsesEachBaseModeLiteral)
{
	for (const auto &[value, expected] : { std::pair{ "auto", mju::BaseMode::kAuto },
	                                       { "fixed", mju::BaseMode::kFixed },
	                                       { "free", mju::BaseMode::kFree } }) {
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.base_mode", value },
		};
		EXPECT_EQ(mju::ParseDescriptionBundleFromMap(flat).base_mode, expected);
	}
}

TEST(DescriptionBundle, RejectsInvalidBaseModeValues)
{
	const std::array<const char *, 5> invalid_values = { "", "Auto", "FIXED", "fixd", " free" };
	for (const char *value : invalid_values) {
		std::map<std::string, std::string> flat = {
			{ "urdf.source", "file" },
			{ "urdf.path", "/tmp/robot.urdf" },
			{ "description.base_mode", value },
		};

		try {
			(void)mju::ParseDescriptionBundleFromMap(flat);
			FAIL() << "description.base_mode must reject value '" << value << "'";
		} catch (const std::runtime_error &error) {
			EXPECT_NE(std::string(error.what()).find("description.base_mode"), std::string::npos);
		}
	}
}

TEST(DescriptionBundle, ParseBaseModeAcceptsExactLiteralsOnly)
{
	EXPECT_EQ(mju::ParseBaseMode("auto"), mju::BaseMode::kAuto);
	EXPECT_EQ(mju::ParseBaseMode("fixed"), mju::BaseMode::kFixed);
	EXPECT_EQ(mju::ParseBaseMode("free"), mju::BaseMode::kFree);
	EXPECT_THROW(mju::ParseBaseMode(""), std::runtime_error);
	EXPECT_THROW(mju::ParseBaseMode("Free"), std::runtime_error);
}
