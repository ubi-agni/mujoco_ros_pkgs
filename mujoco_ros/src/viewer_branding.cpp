#include <mujoco_ros/viewer_branding.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

#include <lodepng.h>
#include <mujoco_ros/ros_version.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <ros/package.h>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace mujoco_ros {
namespace {

std::string MujocoRosSharePath()
{
#if MJR_ROS_VERSION == ROS_1
	const std::string share = ros::package::getPath("mujoco_ros");
	if (share.empty()) {
		throw std::runtime_error("Viewer branding could not locate the mujoco_ros package share directory");
	}
	return share;
#else
	return ament_index_cpp::get_package_share_directory("mujoco_ros");
#endif
}

unsigned ChannelCount(ViewerBrandingPixelFormat format)
{
	return format == ViewerBrandingPixelFormat::kRgb ? 3u : 4u;
}

struct SplashCache
{
	ViewerBrandingImage source;
	std::vector<unsigned char> pixels;
	int viewport_width  = 0;
	int viewport_height = 0;
};

void ResizeSplash(const ViewerBrandingImage &source, int width, int height, std::vector<unsigned char> &output)
{
	const unsigned channels = 3;
	output.assign(static_cast<std::size_t>(width) * height * channels, 0);

	const int scaled_width =
	    std::max(1, std::min(width, static_cast<int>(static_cast<double>(source.width) * height / source.height)));
	const int scaled_height =
	    std::max(1, std::min(height, static_cast<int>(static_cast<double>(source.height) * width / source.width)));
	const int draw_width  = std::min(width, scaled_width);
	const int draw_height = std::min(height, scaled_height);
	const int left        = (width - draw_width) / 2;
	const int top         = (height - draw_height) / 2;

	for (int y = 0; y < draw_height; ++y) {
		const unsigned source_y =
		    static_cast<unsigned>(static_cast<std::size_t>(y) * source.height / static_cast<unsigned>(draw_height));
		const int destination_y = height - 1 - top - y;
		for (int x = 0; x < draw_width; ++x) {
			const unsigned source_x =
			    static_cast<unsigned>(static_cast<std::size_t>(x) * source.width / static_cast<unsigned>(draw_width));
			const auto source_index      = (static_cast<std::size_t>(source_y) * source.width + source_x) * channels;
			const auto destination_index = (static_cast<std::size_t>(destination_y) * width + left + x) * channels;
			std::copy_n(source.pixels.data() + source_index, channels, output.data() + destination_index);
		}
	}
}

} // namespace

ViewerBrandingImage LoadViewerBrandingPngFile(const std::string &path, ViewerBrandingPixelFormat format)
{
	ViewerBrandingImage image;
	const auto color_type = format == ViewerBrandingPixelFormat::kRgb ? LCT_RGB : LCT_RGBA;
	const unsigned error  = lodepng::decode(image.pixels, image.width, image.height, path, color_type, 8);
	if (error != 0) {
		throw std::runtime_error("Viewer branding could not decode '" + path + "': " + lodepng_error_text(error));
	}
	if (image.width == 0 || image.height == 0 ||
	    image.pixels.size() != static_cast<std::size_t>(image.width) * image.height * ChannelCount(format)) {
		throw std::runtime_error("Viewer branding decoded an invalid image: " + path);
	}
	return image;
}

ViewerBrandingImage LoadViewerBrandingAsset(const std::string &filename, ViewerBrandingPixelFormat format)
{
	return LoadViewerBrandingPngFile(MujocoRosSharePath() + "/assets/" + filename, format);
}

void DrawViewerSplash(const mjrRect &viewport, mjrContext *context)
{
	if (viewport.width <= 0 || viewport.height <= 0) {
		return;
	}

	thread_local SplashCache cache;
	if (cache.source.pixels.empty()) {
		cache.source = LoadViewerBrandingAsset("mj_ros_splash.png", ViewerBrandingPixelFormat::kRgb);
	}
	if (cache.viewport_width != viewport.width || cache.viewport_height != viewport.height) {
		ResizeSplash(cache.source, viewport.width, viewport.height, cache.pixels);
		cache.viewport_width  = viewport.width;
		cache.viewport_height = viewport.height;
	}
	mjr_drawPixels(cache.pixels.data(), nullptr, viewport, context);
}

} // namespace mujoco_ros
