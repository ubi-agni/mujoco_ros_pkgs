#include <gtest/gtest.h>

#include <cstddef>
#include <string>

#include <mujoco_ros/viewer_branding.hpp>

namespace mujoco_ros {
namespace {

std::string AssetPath(const char *filename)
{
	return std::string(VIEWER_BRANDING_SOURCE_ASSETS_DIR) + "/" + filename;
}

TEST(ViewerBranding, LoadsIconAsRgba)
{
	const auto image = LoadViewerBrandingPngFile(AssetPath("mj_ros_icon.png"), ViewerBrandingPixelFormat::kRgba);

	EXPECT_GT(image.width, 0u);
	EXPECT_GT(image.height, 0u);
	EXPECT_EQ(image.pixels.size(), static_cast<std::size_t>(image.width) * image.height * 4);
}

TEST(ViewerBranding, LoadsSplashAsRgb)
{
	const auto image = LoadViewerBrandingPngFile(AssetPath("mj_ros_splash.png"), ViewerBrandingPixelFormat::kRgb);

	EXPECT_GT(image.width, 0u);
	EXPECT_GT(image.height, 0u);
	EXPECT_EQ(image.pixels.size(), static_cast<std::size_t>(image.width) * image.height * 3);
}

TEST(ViewerBranding, LoadsInstalledSplashAsset)
{
	const auto image = LoadViewerBrandingAsset("mj_ros_splash.png", ViewerBrandingPixelFormat::kRgb);

	EXPECT_GT(image.width, 0u);
	EXPECT_GT(image.height, 0u);
	EXPECT_EQ(image.pixels.size(), static_cast<std::size_t>(image.width) * image.height * 3);
}

} // namespace
} // namespace mujoco_ros
