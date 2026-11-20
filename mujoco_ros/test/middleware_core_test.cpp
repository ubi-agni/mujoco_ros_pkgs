#include <gtest/gtest.h>

#include <mujoco_ros/middleware/i_config_source.hpp>
#include <mujoco_ros/middleware/i_time_publisher.hpp>
#include <mujoco_ros/middleware/middleware_core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace mujoco_ros::middleware {
namespace {

class FakeConfigSource : public IConfigSource
{
public:
	std::optional<bool> GetBool(const std::string &) const override { return std::nullopt; }
	std::optional<int> GetInt(const std::string &) const override { return std::nullopt; }
	std::optional<double> GetDouble(const std::string &) const override { return std::nullopt; }
	std::optional<std::string> GetString(const std::string &) const override { return std::nullopt; }
	std::optional<std::vector<std::string>> GetStringList(const std::string &) const override { return std::nullopt; }
};

class FakeTimePublisher : public ITimePublisher
{
public:
	void PublishSimTime(mjtNum sim_time) override
	{
		last = sim_time;
		++count;
	}
	mjtNum last = -1.0;
	int count   = 0;
};

TEST(MiddlewareCore, MiddlewareCoreConstructsWithFakePorts)
{
	FakeConfigSource config;
	FakeTimePublisher time_pub;
	MiddlewareCore core(config, time_pub);
	EXPECT_EQ(&core.config_source(), &config);
	EXPECT_EQ(&core.time_publisher(), &time_pub);
	EXPECT_EQ(time_pub.count, 0);
}

} // namespace
} // namespace mujoco_ros::middleware
