#include <gtest/gtest.h>
#include <tinyxml2.h>

TEST(TinyXml2Smoke, ParsesMinimalDocument)
{
	tinyxml2::XMLDocument doc;
	const char *xml = "<root><child attr=\"42\"/></root>";
	ASSERT_EQ(doc.Parse(xml), tinyxml2::XML_SUCCESS);

	auto *child = doc.RootElement()->FirstChildElement("child");
	ASSERT_NE(child, nullptr);
	EXPECT_EQ(child->IntAttribute("attr"), 42);
}
