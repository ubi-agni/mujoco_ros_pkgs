/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <gtest/gtest.h>

#include <mujoco_ros/ros_version.hpp>
#if MJR_ROS_VERSION == ROS_2
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#endif

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

#if MJR_ROS_VERSION == ROS_2
namespace {

constexpr char kJointVelXml[] = R"(
<mujoco>
  <worldbody><body><joint name="j" type="hinge" axis="0 1 0"/><geom size="0.1"/></body></worldbody>
  <sensor><jointvel name="jv" joint="j"/></sensor>
</mujoco>)";

// Noisy scalar stream over a real model; seeded noise makes sample consumption observable.
struct NoisyScalar
{
	NoisyScalar()
	{
		char err[256] = "";
		mjSpec *spec  = mj_parseXMLString(kJointVelXml, nullptr, err, sizeof(err));
		EXPECT_NE(spec, nullptr) << err;
		model = mj_compile(spec, nullptr);
		mj_deleteSpec(spec);
		data = mj_makeData(model);
		data->qvel[0] = 1.0;
		mj_forward(model, data);
		config.frame_id = "f";
		config.is_set   = 1;
		config.mean[0]  = 0.0;
		config.sigma[0] = 1.0;
		noise.generator.seed(42);
		stream = MakeNativeStream(model, 0, "jv", config, noise, false);
	}
	~NoisyScalar()
	{
		mj_deleteData(data);
		mj_deleteModel(model);
	}
	double Value() { return std::any_cast<const mujoco_ros_msgs::msg::ScalarStamped &>(stream->Value(model, data)).value; }

	mjModel *model = nullptr;
	mjData *data   = nullptr;
	SensorConfig config;
	NoiseSource noise;
	std::unique_ptr<ISensorStream> stream;
};

} // namespace

TEST(StreamCacheTest, TwoReadsInOneStepAreEqualWithNoise)
{
	NoisyScalar s;
	const double first = s.Value();
	EXPECT_EQ(first, s.Value());
	EXPECT_NE(first, 1.0) << "noise must actually be applied";
	s.stream->InvalidateCache();
	EXPECT_NE(first, s.Value()) << "a new step must draw new noise";
}

TEST(StreamCacheTest, ExtraReadsDoNotShiftNoiseSequence)
{
	// "Python" reads three times per step, "ROS-only" once: the per-step values must still match.
	NoisyScalar many;
	NoisyScalar once;
	for (int step = 0; step < 4; ++step) {
		many.stream->InvalidateCache();
		once.stream->InvalidateCache();
		many.Value();
		many.Value();
		EXPECT_EQ(many.Value(), once.Value()) << "step " << step;
	}
}

TEST(StreamCacheTest, ResetStyleInvalidationRebuilds)
{
	NoisyScalar s;
	const double a = s.Value();
	s.stream->InvalidateCache();
	EXPECT_NE(a, s.Value());
}

TEST(StreamCacheTest, GroundTruthIsCachedAndDoesNotUseNoise)
{
	NoisyScalar s;
	const auto &gt1 = std::any_cast<const mujoco_ros_msgs::msg::ScalarStamped &>(s.stream->GroundTruth(s.model, s.data));
	EXPECT_NEAR(gt1.value, 1.0, 1e-6);
	// Noisy sequence must be untouched by the GT read.
	NoisyScalar fresh;
	EXPECT_EQ(s.Value(), fresh.Value());
}

TEST(StreamCacheTest, GroundTruthThrowsInEvalMode)
{
	NoisyScalar s;
	auto eval = MakeNativeStream(s.model, 0, "jv", s.config, s.noise, true);
	EXPECT_THROW(eval->GroundTruth(s.model, s.data), std::runtime_error);
}
#endif
