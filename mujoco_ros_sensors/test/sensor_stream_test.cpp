/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros_sensors/native_sensor_streams.hpp>
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

namespace {

mjModel *CompileCameraWorld()
{
	static const char *xml = R"(
<mujoco>
  <worldbody>
    <camera name="cam" pos="0 -2 1" xyaxes="1 0 0 0 0.5 1"/>
    <body name="b"><geom size=".1"/><site name="s" pos="0 0 .1"/></body>
    <body><geom size=".1"/><site name="orphan_site" pos="0 0 .1"/></body>
  </worldbody>
  <sensor>
    <camprojection name="proj" site="s" camera="cam"/>
    <insidesite name="inside" site="s" objtype="xbody" objname="b"/>
    <insidesite name="inside_orphan" site="orphan_site" objtype="xbody" objname="b"/>
  </sensor>
</mujoco>)";
	char error[1000]       = "";
	mjSpec *spec           = mj_parseXMLString(xml, nullptr, error, sizeof(error));
	EXPECT_NE(spec, nullptr) << error;
	mjModel *model = spec ? mj_compile(spec, nullptr) : nullptr;
	if (spec) {
		mj_deleteSpec(spec);
	}
	return model;
}

} // namespace

TEST(GapFrameTest, ResolvesCameraAndRefSiteBody)
{
	mjModel *model = CompileCameraWorld();
	ASSERT_NE(model, nullptr);
	std::string frame, error;
	EXPECT_TRUE(ResolveGapFrameId(model, 0, frame, error)) << error;
	EXPECT_EQ(frame, "cam");
	EXPECT_TRUE(ResolveGapFrameId(model, 1, frame, error)) << error;
	EXPECT_EQ(frame, "b");
	mj_deleteModel(model);
}

TEST(GapFrameTest, UnnamedRefSiteBodyFails)
{
	mjModel *model = CompileCameraWorld();
	ASSERT_NE(model, nullptr);
	std::string frame = "untouched", error;
	EXPECT_FALSE(ResolveGapFrameId(model, 2, frame, error));
	EXPECT_FALSE(error.empty());
	mj_deleteModel(model);
}

TEST(GapFrameTest, MissingCameraRefFails)
{
	mjModel *model = CompileCameraWorld();
	ASSERT_NE(model, nullptr);
	// MJCF cannot reference a missing camera, so emulate a model whose ref is gone.
	model->sensor_refid[0] = -1;
	std::string frame, error;
	EXPECT_FALSE(ResolveGapFrameId(model, 0, frame, error));
	EXPECT_FALSE(error.empty());
	mj_deleteModel(model);
}

TEST(GapFrameTest, UnnamedCameraFails)
{
	mjModel *model = CompileCameraWorld();
	ASSERT_NE(model, nullptr);
	// Blank the camera's name in place; MJCF cannot reference an unnamed camera.
	model->names[model->name_camadr[0]] = '\0';
	std::string frame, error;
	EXPECT_FALSE(ResolveGapFrameId(model, 0, frame, error));
	mj_deleteModel(model);
}

TEST(GapFrameTest, IgnoreAndDimTables)
{
	for (int type : { mjSENS_CLOCK, mjSENS_CONTACT, mjSENS_TACTILE, mjSENS_PLUGIN, mjSENS_USER }) {
		EXPECT_TRUE(IsIgnoredSensorType(type)) << type;
		EXPECT_EQ(ExpectedNativeDim(type), -1) << type;
	}
	EXPECT_EQ(ExpectedNativeDim(mjSENS_GEOMFROMTO), 6);
	EXPECT_EQ(ExpectedNativeDim(mjSENS_CAMPROJECTION), 2);
	EXPECT_EQ(ExpectedNativeDim(mjSENS_FRAMEQUAT), 4);
	EXPECT_FALSE(IsIgnoredSensorType(mjSENS_GEOMDIST));
}
