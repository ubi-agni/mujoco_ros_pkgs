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

#include <mujoco_ros_sensors/compound_sensors.hpp>
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

namespace {

mjModel *CompileCompoundWorld()
{
	static const char *xml = R"(
<mujoco>
  <worldbody>
    <body name="a"><geom size=".1"/><site name="sa" pos="0 0 .1"/></body>
    <body name="b"><geom size=".1"/><joint name="ball_b" type="ball"/><site name="sb" pos="0 0 .1"/></body>
    <body name="c"><geom size=".1"/><joint name="ball_a" type="ball"/></body>
  </worldbody>
  <sensor>
    <accelerometer name="acc_a" site="sa"/>
    <framequat name="quat_a" objtype="site" objname="sa"/>
    <framequat name="quat_a_body" objtype="xbody" objname="a"/>
    <framequat name="quat_b" objtype="xbody" objname="b"/>
    <ballquat name="bq_b" joint="ball_b"/>
    <ballquat name="bq_c" joint="ball_a"/>
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

int SensorId(const mjModel *model, const char *name)
{
	return mj_name2id(model, mjOBJ_SENSOR, name);
}

} // namespace

TEST(CompoundKindTest, ParsesExactYamlTypeStrings)
{
	CompoundKind kind;
	EXPECT_TRUE(ParseCompoundKind("IMU", kind));
	EXPECT_EQ(kind, CompoundKind::kIMU);
	EXPECT_TRUE(ParseCompoundKind("Twist", kind));
	EXPECT_EQ(kind, CompoundKind::kTwist);
	EXPECT_TRUE(ParseCompoundKind("Wrench", kind));
	EXPECT_EQ(kind, CompoundKind::kWrench);
	EXPECT_TRUE(ParseCompoundKind("Pose", kind));
	EXPECT_EQ(kind, CompoundKind::kPose);
	EXPECT_TRUE(ParseCompoundKind("Accel", kind));
	EXPECT_EQ(kind, CompoundKind::kAccel);
	EXPECT_TRUE(ParseCompoundKind("Odometry", kind));
	EXPECT_EQ(kind, CompoundKind::kOdometry);
}

TEST(CompoundKindTest, RejectsNonExactStrings)
{
	CompoundKind kind = CompoundKind::kPose;
	for (const char *bad : { "IMUSensor", "imu", "Imu", "", "Wrench ", "Odom" }) {
		EXPECT_FALSE(ParseCompoundKind(bad, kind)) << bad;
	}
	EXPECT_EQ(kind, CompoundKind::kPose);
}

TEST(CompoundRolesTest, RequiredRolesPerKind)
{
	using V = std::vector<std::string>;
	EXPECT_EQ(RequiredRoles(CompoundKind::kIMU), (V{ "linear_acceleration", "angular_velocity", "orientation" }));
	EXPECT_EQ(RequiredRoles(CompoundKind::kTwist), (V{ "linear", "angular" }));
	EXPECT_EQ(RequiredRoles(CompoundKind::kWrench), (V{ "force", "torque" }));
	EXPECT_EQ(RequiredRoles(CompoundKind::kPose), (V{ "position", "orientation" }));
	EXPECT_EQ(RequiredRoles(CompoundKind::kAccel), (V{ "linear", "angular" }));
	EXPECT_EQ(RequiredRoles(CompoundKind::kOdometry), (V{ "position", "orientation", "linear", "angular" }));
}

TEST(CompoundRolesTest, WrenchForceAndTorqueTypesDiffer)
{
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kWrench, "force", mjSENS_FORCE));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kWrench, "force", mjSENS_TORQUE));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kWrench, "torque", mjSENS_TORQUE));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kWrench, "torque", mjSENS_FORCE));
}

TEST(CompoundRolesTest, ImuRoleTypeSets)
{
	const auto k = CompoundKind::kIMU;
	EXPECT_TRUE(RoleAcceptsMjType(k, "linear_acceleration", mjSENS_ACCELEROMETER));
	EXPECT_TRUE(RoleAcceptsMjType(k, "linear_acceleration", mjSENS_FRAMELINACC));
	EXPECT_FALSE(RoleAcceptsMjType(k, "linear_acceleration", mjSENS_GYRO));
	EXPECT_TRUE(RoleAcceptsMjType(k, "angular_velocity", mjSENS_GYRO));
	EXPECT_TRUE(RoleAcceptsMjType(k, "angular_velocity", mjSENS_FRAMEANGVEL));
	EXPECT_TRUE(RoleAcceptsMjType(k, "angular_velocity", mjSENS_BALLANGVEL));
	EXPECT_FALSE(RoleAcceptsMjType(k, "angular_velocity", mjSENS_ACCELEROMETER));
	EXPECT_TRUE(RoleAcceptsMjType(k, "orientation", mjSENS_FRAMEQUAT));
	EXPECT_TRUE(RoleAcceptsMjType(k, "orientation", mjSENS_BALLQUAT));
	EXPECT_FALSE(RoleAcceptsMjType(k, "orientation", mjSENS_FRAMEPOS));
}

TEST(CompoundRolesTest, TwistPoseAccelAndOdometrySets)
{
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kTwist, "linear", mjSENS_FRAMELINVEL));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kTwist, "linear", mjSENS_FRAMELINACC));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kTwist, "angular", mjSENS_FRAMEANGVEL));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kPose, "position", mjSENS_FRAMEPOS));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kPose, "position", mjSENS_FRAMELINVEL));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kPose, "orientation", mjSENS_BALLQUAT));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kAccel, "linear", mjSENS_FRAMELINACC));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kAccel, "angular", mjSENS_FRAMEANGACC));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kAccel, "angular", mjSENS_FRAMEANGVEL));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kOdometry, "position", mjSENS_FRAMEPOS));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kOdometry, "orientation", mjSENS_FRAMEQUAT));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kOdometry, "linear", mjSENS_FRAMELINVEL));
	EXPECT_TRUE(RoleAcceptsMjType(CompoundKind::kOdometry, "angular", mjSENS_FRAMEANGVEL));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kOdometry, "angular", mjSENS_GYRO));
}

TEST(CompoundRolesTest, UnknownRoleRejected)
{
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kWrench, "linear", mjSENS_FORCE));
	EXPECT_FALSE(RoleAcceptsMjType(CompoundKind::kPose, "", mjSENS_FRAMEPOS));
}

TEST(CompoundBodyIdTest, ResolvesSiteBodyAndBallJointBody)
{
	mjModel *model = CompileCompoundWorld();
	ASSERT_NE(model, nullptr);
	int body = -1;
	std::string error;
	const int body_a = mj_name2id(model, mjOBJ_BODY, "a");
	const int body_b = mj_name2id(model, mjOBJ_BODY, "b");
	EXPECT_TRUE(ResolveSensorBodyId(model, SensorId(model, "acc_a"), body, error)) << error;
	EXPECT_EQ(body, body_a);
	EXPECT_TRUE(ResolveSensorBodyId(model, SensorId(model, "quat_a_body"), body, error)) << error;
	EXPECT_EQ(body, body_a);
	EXPECT_TRUE(ResolveSensorBodyId(model, SensorId(model, "bq_b"), body, error)) << error;
	EXPECT_EQ(body, body_b);
	mj_deleteModel(model);
}

TEST(CompoundBodyIdTest, InvalidSensorIdFails)
{
	mjModel *model = CompileCompoundWorld();
	ASSERT_NE(model, nullptr);
	int body = -2;
	std::string error;
	EXPECT_FALSE(ResolveSensorBodyId(model, -1, body, error));
	EXPECT_FALSE(error.empty());
	EXPECT_FALSE(ResolveSensorBodyId(model, model->nsensor, body, error));
	mj_deleteModel(model);
}

TEST(CompoundBodyIdTest, SensorWithoutBodyObjectFails)
{
	mjModel *model = CompileCompoundWorld();
	ASSERT_NE(model, nullptr);
	// Emulate an object type with no body owner.
	const int id = SensorId(model, "quat_a_body");
	model->sensor_objtype[id] = mjOBJ_UNKNOWN;
	int body = -2;
	std::string error;
	EXPECT_FALSE(ResolveSensorBodyId(model, id, body, error));
	EXPECT_FALSE(error.empty());
	mj_deleteModel(model);
}

TEST(CompoundAnchorTest, SameBodyMatchesAcrossSiteAndXbody)
{
	mjModel *model = CompileCompoundWorld();
	ASSERT_NE(model, nullptr);
	std::string error;
	EXPECT_TRUE(SameAnchorBody(model, SensorId(model, "acc_a"), SensorId(model, "quat_a"), error)) << error;
	EXPECT_TRUE(SameAnchorBody(model, SensorId(model, "acc_a"), SensorId(model, "quat_a_body"), error)) << error;
	mj_deleteModel(model);
}

TEST(CompoundAnchorTest, MismatchedBallQuatFails)
{
	mjModel *model = CompileCompoundWorld();
	ASSERT_NE(model, nullptr);
	std::string error;
	// ball_a lives on body "c", not on the anchor body "a".
	EXPECT_FALSE(SameAnchorBody(model, SensorId(model, "acc_a"), SensorId(model, "bq_c"), error));
	EXPECT_FALSE(error.empty());
	error.clear();
	EXPECT_FALSE(SameAnchorBody(model, SensorId(model, "acc_a"), SensorId(model, "quat_b"), error));
	EXPECT_FALSE(error.empty());
	mj_deleteModel(model);
}
