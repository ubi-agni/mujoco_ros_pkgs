#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mujoco_ros {

enum class ViewerBrandingPixelFormat
{
	kRgb,
	kRgba,
};

struct ViewerBrandingImage
{
	unsigned width  = 0;
	unsigned height = 0;
	std::vector<unsigned char> pixels;
};

ViewerBrandingImage LoadViewerBrandingPngFile(const std::string &path, ViewerBrandingPixelFormat format);
ViewerBrandingImage LoadViewerBrandingAsset(const std::string &filename, ViewerBrandingPixelFormat format);

void DrawViewerSplash(const mjrRect &viewport, mjrContext *context);

} // namespace mujoco_ros
