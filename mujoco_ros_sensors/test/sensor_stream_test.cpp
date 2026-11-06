/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros_sensors/sensor_stream.hpp>

using namespace mujoco_ros::sensors;

namespace {

class CountingStream : public ISensorStream
{
public:
	using ISensorStream::ISensorStream;
	void MaybePublish(const mjModel *, const mjData *) override { ++published; }
	std::any BuildValue(const mjModel *, const mjData *) override { return 42.0; }
	int published = 0;
};

class WritingStream : public CountingStream
{
public:
	using CountingStream::CountingStream;
	bool ObservesOnly() const override { return false; }
};

class StreamProvider : public mujoco_ros::ISensorStreamProvider
{
public:
	explicit StreamProvider(ISensorStream &stream) : stream_(stream) {}
	mujoco_ros::SensorProviderKind kind() const override { return mujoco_ros::SensorProviderKind::kNative; }
	mujoco_ros::SensorStreamBase *Find(const std::string &name) const override
	{
		return (stream_.is_public() && stream_.name() == name) ? &stream_ : nullptr;
	}
	std::vector<std::string> PublicNames() const override
	{
		return stream_.is_public() ? std::vector<std::string>{ stream_.name() } : std::vector<std::string>{};
	}

private:
	ISensorStream &stream_;
};

} // namespace

TEST(SensorHubTest, DefaultObservesOnlyTrue)
{
	SensorConfig config;
	CountingStream observing("a", config, true);
	WritingStream writing("b", config, true);
	EXPECT_TRUE(observing.ObservesOnly());
	EXPECT_FALSE(writing.ObservesOnly());
}

TEST(SensorHubTest, SensorConfigUpdateRateDefaultsToEveryStep)
{
	SensorConfig config;
	EXPECT_EQ(config.update_rate, 0.0);
}

TEST(SensorHubTest, StreamReferencesItsConfigAndExposesIdentity)
{
	SensorConfig config("frame");
	CountingStream stream("accel", config, true);
	EXPECT_EQ(&stream.config(), &config);
	EXPECT_EQ(stream.name(), "accel");
	EXPECT_EQ(stream.config().frame_id, "frame");
	EXPECT_DOUBLE_EQ(std::any_cast<double>(stream.BuildValue(nullptr, nullptr)), 42.0);
}

TEST(SensorHubTest, HubReturnsStreamFromProviderAndHidesNonPublic)
{
	SensorConfig config;
	CountingStream visible("visible", config, true);
	CountingStream hidden("hidden", config, false);
	StreamProvider visible_provider(visible);
	StreamProvider hidden_provider(hidden);

	mujoco_ros::SensorHub hub;
	hub.RegisterProvider(visible_provider);
	hub.RegisterProvider(hidden_provider);

	EXPECT_EQ(hub.Native().Get("visible"), &visible);
	EXPECT_EQ(hub.Get("hidden"), nullptr);
}
