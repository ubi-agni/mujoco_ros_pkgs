#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <mujoco_ros/extended_params.hpp>

namespace mju = mujoco_ros;

namespace {

std::filesystem::path WriteTemporaryXml(const std::string &filename, const std::string &contents)
{
	const auto path = std::filesystem::temp_directory_path() / filename;
	std::ofstream xml(path);
	if (!xml.is_open())
		throw std::runtime_error("Failed to create temporary XML fixture");
	xml << contents;
	return path;
}

} // namespace

TEST(ExtendedParams, ParsesDisabledCollisionsAndIgnoresGroups)
{
	auto exclusions = mju::ParseDisabledCollisions(std::string(TEST_RESOURCES_DIR) + "/srdf_two_exclusions.srdf");

	ASSERT_EQ(exclusions.size(), 2u);
	EXPECT_EQ(exclusions[0].link1, "link_a");
	EXPECT_EQ(exclusions[0].link2, "link_b");
	EXPECT_EQ(exclusions[1].link1, "link_b");
	EXPECT_EQ(exclusions[1].link2, "link_c");
}

TEST(ExtendedParams, ThrowsOnDisabledCollisionsMissingLinkAttribute)
{
	// <disable_collisions> without required "link2" must fail loudly, not silently skip.
	const std::string bad_srdf_path = std::string(TEST_RESOURCES_DIR) + "/srdf_malformed_exclusion.srdf";
	EXPECT_THROW(mju::ParseDisabledCollisions(bad_srdf_path), std::runtime_error);
}

TEST(ExtendedParams, ParsesSrdfExtendedParamsAndDisableCollisionsTogether)
{
	auto parsed = mju::ParseSrdfExtensions(std::string(TEST_RESOURCES_DIR) + "/srdf_extended_params_canonical.srdf");

	ASSERT_EQ(parsed.collision_exclusions.size(), 2u);
	ASSERT_EQ(parsed.entries.size(), 1u);
	const auto &entry = parsed.entries.at("joint1");
	ASSERT_TRUE(entry.joint_params.has_value());
	ASSERT_TRUE(entry.joint_params->actuator.has_value());
	EXPECT_DOUBLE_EQ(*entry.joint_params->actuator->kp, 121.0);
	ASSERT_TRUE(entry.gravcomp.has_value());
	EXPECT_DOUBLE_EQ(entry.gravcomp->value, 1.0);
}

TEST(ExtendedParams, RemovesBuiltInChildrenBeforeCustomDispatch)
{
	auto parsed =
	    mju::ParseSrdfExtensions(std::string(TEST_RESOURCES_DIR) + "/srdf_extended_params_with_custom_tags.srdf");

	const auto *xml = parsed.entries.at("joint1").custom_xml;
	ASSERT_NE(xml, nullptr);
	EXPECT_EQ(xml->FirstChildElement("mujoco_gravcomp"), nullptr);
	ASSERT_NE(xml->FirstChildElement("vendor_tag"), nullptr);
}

TEST(ExtendedParams, KeepsFrictionMotorAndComplianceAsCustomXmlSinceTheyAreNoLongerBuiltIn)
{
	auto parsed = mju::ParseSrdfExtensions(std::string(TEST_RESOURCES_DIR) + "/srdf_extended_params_canonical.srdf");

	const auto *xml = parsed.entries.at("joint1").custom_xml;
	ASSERT_NE(xml, nullptr);
	EXPECT_NE(xml->FirstChildElement("motor"), nullptr) << "motor is no longer built-in, must survive as custom XML";
	EXPECT_EQ(xml->FirstChildElement("mujoco_gravcomp"), nullptr)
	    << "gravcomp is still built-in, must still be stripped";
	EXPECT_EQ(xml->FirstChildElement("mujoco_actuator"), nullptr)
	    << "actuator is still built-in, must still be stripped";
}

TEST(ExtendedParams, RejectsNonFiniteSrdfBuiltInNumericValues)
{
	const auto xml_path = std::filesystem::temp_directory_path() / "mujoco_ros_nonfinite_extended_params.srdf";
	{
		std::ofstream xml(xml_path);
		ASSERT_TRUE(xml.is_open());
		xml << R"(<robot name="extended_params_nonfinite">
  <extended_params name="joint1">
    <mujoco_actuator kp="nan"/>
  </extended_params>
</robot>)";
	}

	EXPECT_THROW(mju::ParseSrdfExtensions(xml_path.string()), std::runtime_error);
	std::filesystem::remove(xml_path);
}

TEST(ExtendedParams, RejectsDuplicateSrdfBuiltInTags)
{
	const std::pair<const char *, const char *> cases[] = {
		{ "mujoco_actuator", R"(<mujoco_actuator kp="1"/><mujoco_actuator kp="2"/>)" },
		{ "mujoco_gravcomp", R"(<mujoco_gravcomp value="1"/><mujoco_gravcomp value="2"/>)" },
	};

	for (const auto &[name, children] : cases) {
		const auto xml_path = WriteTemporaryXml(std::string("mujoco_ros_duplicate_") + name + ".srdf",
		                                        std::string("<robot><extended_params name=\"joint1\">") + children +
		                                            "</extended_params></robot>");
		EXPECT_THROW(mju::ParseSrdfExtensions(xml_path.string()), std::runtime_error) << name;
		std::filesystem::remove(xml_path);
	}
}

TEST(ExtendedParams, RejectsUnexpectedParserRootElements)
{
	const auto srdf_path =
	    WriteTemporaryXml("mujoco_ros_invalid_srdf_root.xml",
	                      "<extended_params name=\"joint1\"><mujoco_gravcomp value=\"1\"/></extended_params>");
	EXPECT_THROW(mju::ParseSrdfExtensions(srdf_path.string()), std::runtime_error);
	EXPECT_THROW(mju::ParseDisabledCollisions(srdf_path.string()), std::runtime_error);
	std::filesystem::remove(srdf_path);
}

TEST(ExtendedParams, ThrowsOnMissingSrdfFile)
{
	EXPECT_THROW(mju::ParseSrdfExtensions("/nonexistent/path.srdf"), std::runtime_error);
}

TEST(ExtendedParams, RejectsBlankSrdfExtensionAndCollisionAttributes)
{
	const auto cases = {
		std::string("<robot><extended_params name=\"\"/></robot>"),
		std::string("<robot><disable_collisions link1=\"\" link2=\"link_1\"/></robot>"),
		std::string("<robot><disable_collisions link1=\"base_link\" link2=\"\"/></robot>"),
	};
	for (const auto &xml : cases) {
		const auto path = WriteTemporaryXml("mujoco_ros_blank_srdf_attribute.srdf", xml);
		EXPECT_THROW(mju::ParseSrdfExtensions(path.string()), std::runtime_error) << xml;
		std::filesystem::remove(path);
	}
}

TEST(ExtendedParams, LeavesJointParamsUnsetForGravcompOnlyEntry)
{
	const auto xml_path = WriteTemporaryXml(
	    "mujoco_ros_gravcomp_only.srdf",
	    "<robot><extended_params name=\"joint1\"><mujoco_gravcomp value=\"1\"/></extended_params></robot>");

	auto parsed = mju::ParseSrdfExtensions(xml_path.string());
	ASSERT_TRUE(parsed.entries.at("joint1").gravcomp.has_value());
	EXPECT_FALSE(parsed.entries.at("joint1").joint_params.has_value());
	std::filesystem::remove(xml_path);
}

TEST(ExtendedParams, SkipsUnknownCustomTagsAndContinuesDispatchingRegisteredOnes)
{
	mju::ExtendedParamsEntry entry;
	entry.custom_doc = std::make_unique<tinyxml2::XMLDocument>();
	ASSERT_EQ(
	    entry.custom_doc->Parse("<extended_params name=\"joint1\"><unregistered_tag/><vendor_tag/></extended_params>"),
	    tinyxml2::XML_SUCCESS);
	entry.custom_xml = entry.custom_doc->RootElement();

	mju::ExtendedParamsHandlerRegistry registry;
	bool called = false;
	registry.Register("vendor_tag", [&called](const std::string &, const std::string &, const tinyxml2::XMLElement &,
	                                          mju::ConverterExtensionContext &) { called = true; });

	// The unregistered "unregistered_tag" comes first in document order --
	// it must be skipped (not thrown, not a stop-the-world abort), and
	// dispatch must continue on to "vendor_tag" right after it.
	ASSERT_NO_THROW(mju::DispatchCustomExtendedParams("joint1", entry, registry));
	EXPECT_TRUE(called);
}

TEST(ExtendedParams, DispatchesSanitizedCustomTagsToRegisteredHandlerWithJointName)
{
	mju::ExtendedParamsEntry entry;
	entry.custom_doc = std::make_unique<tinyxml2::XMLDocument>();
	ASSERT_EQ(entry.custom_doc->Parse("<extended_params name=\"joint1\"><vendor_tag/></extended_params>"),
	          tinyxml2::XML_SUCCESS);
	entry.custom_xml = entry.custom_doc->RootElement();

	mju::ExtendedParamsHandlerRegistry registry;
	bool called = false;
	registry.Register("vendor_tag", [&called](const std::string &joint_name, const std::string &tag_name,
	                                          const tinyxml2::XMLElement &xml, mju::ConverterExtensionContext &) {
		called = joint_name == "joint_1" && tag_name == "vendor_tag" && std::string(xml.Name()) == "vendor_tag";
	});

	mju::DispatchCustomExtendedParams("joint_1", entry, registry);
	EXPECT_TRUE(called);
}

TEST(ExtendedParams, GlobalRegistryRegistersAndFindsAHandlerRoundTrip)
{
	bool called = false;
	mju::RegisterExtendedParamsHandler("global_registry_round_trip_tag",
	                                   [&called](const std::string &, const std::string &, const tinyxml2::XMLElement &,
	                                             mju::ConverterExtensionContext &) { called = true; });

	const auto *handler = mju::GetGlobalExtendedParamsRegistry().Find("global_registry_round_trip_tag");
	ASSERT_NE(handler, nullptr);

	tinyxml2::XMLDocument doc;
	ASSERT_EQ(doc.Parse("<global_registry_round_trip_tag/>"), tinyxml2::XML_SUCCESS);
	mju::ConverterExtensionContext context;
	(*handler)("joint_1", "global_registry_round_trip_tag", *doc.RootElement(), context);
	EXPECT_TRUE(called);
}

TEST(ExtendedParams, GlobalRegistryThrowsOnDuplicateTagRegistration)
{
	mju::RegisterExtendedParamsHandler(
	    "global_registry_duplicate_tag",
	    [](const std::string &, const std::string &, const tinyxml2::XMLElement &, mju::ConverterExtensionContext &) {});
	EXPECT_THROW(
	    mju::RegisterExtendedParamsHandler("global_registry_duplicate_tag",
	                                       [](const std::string &, const std::string &, const tinyxml2::XMLElement &,
	                                          mju::ConverterExtensionContext &) {}),
	    std::runtime_error);
}
