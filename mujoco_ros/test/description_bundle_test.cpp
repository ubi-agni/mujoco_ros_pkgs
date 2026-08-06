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
