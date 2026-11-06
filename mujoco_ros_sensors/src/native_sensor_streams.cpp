/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2026, Bielefeld University
 *  All rights reserved.
 */

#include <mujoco_ros_sensors/native_sensor_streams.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <mujoco_ros/util.hpp>

#include <array>

namespace mujoco_ros::sensors {

#if MJR_ROS_VERSION == ROS_1
using PointStamped      = geometry_msgs::PointStamped;
using QuaternionStamped = geometry_msgs::QuaternionStamped;
using ScalarStamped     = mujoco_ros_msgs::ScalarStamped;
using Vector3Stamped    = geometry_msgs::Vector3Stamped;

template <typename MessageT>
void Publish(const ros::Publisher &publisher, const MessageT &message)
{
	publisher.publish(message);
}
#else
using PointStamped      = geometry_msgs::msg::PointStamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
using ScalarStamped     = mujoco_ros_msgs::msg::ScalarStamped;
using Vector3Stamped    = geometry_msgs::msg::Vector3Stamped;

template <typename MessageT>
void Publish(const std::any &publisher, const MessageT &message)
{
	std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(publisher)->publish(message);
}
#endif

namespace {

/**
 * Publishes the (possibly noisy) value and, outside eval mode, the ground
 * truth. The publisher-exists-but-nobody-listens gate is not applied yet:
 * every step publishes, as before the stream split.
 */
template <typename MsgT>
class NativeStream : public ISensorStream
{
public:
	NativeStream(std::string name, const SensorConfig &config, int sensor_id, NoiseSource &noise, bool eval_mode)
	    : ISensorStream(std::move(name), config, true), sensor_id_(sensor_id), noise_(noise), eval_mode_(eval_mode)
	{
	}

	void MaybePublish(const mjModel *model, const mjData *data) override
	{
		Publish(config().value_pub, Noisy(model, data));
		if (!eval_mode_) {
			Publish(config().gt_pub, Truth(model, data));
		}
	}

	std::any BuildValue(const mjModel *model, const mjData *data) override { return Noisy(model, data); }

protected:
	virtual MsgT Noisy(const mjModel *model, const mjData *data) = 0;
	virtual MsgT Truth(const mjModel *model, const mjData *data) = 0;

	int Adr(const mjModel *model) const { return model->sensor_adr[sensor_id_]; }
	double Cutoff(const mjModel *model) const
	{
		return model->sensor_cutoff[sensor_id_] > 0 ? model->sensor_cutoff[sensor_id_] : 1;
	}

	template <typename HeaderT>
	void Stamp(HeaderT &header, const mjData *data) const
	{
		header.frame_id = config().frame_id;
		header.stamp    = util::toRosTime(util::simTimeToNanoseconds(data->time));
	}

	/// Noise parameters are consumed in order of set axes, not by axis index.
	std::array<double, 3> NoisyXYZ(const mjModel *model, const mjData *data) const
	{
		const SensorConfig &cfg = config();
		const int adr           = Adr(model);
		const double cutoff     = Cutoff(model);
		int noise_idx           = 0;
		std::array<double, 3> v{};
		for (int i = 0; i < 3; i++) {
			double noise = 0.0;
			if (cfg.is_set & (1 << i)) {
				noise = noise_.Sample() * cfg.sigma[noise_idx] + cfg.mean[noise_idx];
				noise_idx += 1;
			}
			v[i] = static_cast<float>(data->sensordata[adr + i] + noise) / cutoff;
		}
		return v;
	}

	std::array<double, 3> TruthXYZ(const mjModel *model, const mjData *data) const
	{
		const int adr       = Adr(model);
		const double cutoff = Cutoff(model);
		return {static_cast<float>(data->sensordata[adr] / cutoff), static_cast<float>(data->sensordata[adr + 1] / cutoff),
		        static_cast<float>(data->sensordata[adr + 2] / cutoff)};
	}

	int sensor_id_;
	NoiseSource &noise_;
	bool eval_mode_;
};

class Vector3Stream : public NativeStream<Vector3Stamped>
{
public:
	using NativeStream::NativeStream;

protected:
	Vector3Stamped Noisy(const mjModel *model, const mjData *data) override { return Make(data, NoisyXYZ(model, data)); }
	Vector3Stamped Truth(const mjModel *model, const mjData *data) override { return Make(data, TruthXYZ(model, data)); }

private:
	Vector3Stamped Make(const mjData *data, const std::array<double, 3> &v) const
	{
		Vector3Stamped msg;
		Stamp(msg.header, data);
		msg.vector.x = v[0];
		msg.vector.y = v[1];
		msg.vector.z = v[2];
		return msg;
	}
};

class PointStream : public NativeStream<PointStamped>
{
public:
	using NativeStream::NativeStream;

protected:
	PointStamped Noisy(const mjModel *model, const mjData *data) override { return Make(data, NoisyXYZ(model, data)); }
	PointStamped Truth(const mjModel *model, const mjData *data) override { return Make(data, TruthXYZ(model, data)); }

private:
	PointStamped Make(const mjData *data, const std::array<double, 3> &v) const
	{
		PointStamped msg;
		Stamp(msg.header, data);
		msg.point.x = v[0];
		msg.point.y = v[1];
		msg.point.z = v[2];
		return msg;
	}
};

class ScalarStream : public NativeStream<ScalarStamped>
{
public:
	using NativeStream::NativeStream;

protected:
	ScalarStamped Noisy(const mjModel *model, const mjData *data) override
	{
		double noise = 0.0;
		if (config().is_set & 0x01) {
			noise = noise_.Sample() * config().sigma[0] + config().mean[0];
		}
		ScalarStamped msg;
		Stamp(msg.header, data);
		msg.value = data->sensordata[Adr(model)] + noise / Cutoff(model);
		return msg;
	}

	ScalarStamped Truth(const mjModel *model, const mjData *data) override
	{
		ScalarStamped msg;
		Stamp(msg.header, data);
		msg.value = data->sensordata[Adr(model)] / Cutoff(model);
		return msg;
	}
};

class QuaternionStream : public NativeStream<QuaternionStamped>
{
public:
	using NativeStream::NativeStream;

protected:
	QuaternionStamped Truth(const mjModel *model, const mjData *data) override
	{
		const int adr       = Adr(model);
		const double cutoff = Cutoff(model);
		QuaternionStamped msg;
		Stamp(msg.header, data);
		msg.quaternion.w = data->sensordata[adr] / cutoff;
		msg.quaternion.x = data->sensordata[adr + 1] / cutoff;
		msg.quaternion.y = data->sensordata[adr + 2] / cutoff;
		msg.quaternion.z = data->sensordata[adr + 3] / cutoff;
		return msg;
	}

	QuaternionStamped Noisy(const mjModel *model, const mjData *data) override
	{
		QuaternionStamped msg = Truth(model, data);
		const SensorConfig &cfg = config();
		if (cfg.is_set == 0) {
			return msg;
		}

		tf2::Quaternion q_orig;
		tf2::fromMsg(msg.quaternion, q_orig);
		q_orig.normalize();

		int noise_idx = 0;
		double r = 0.0, p = 0.0, y = 0.0;
		if (cfg.is_set & 0x01) {
			r = noise_.Sample() * cfg.sigma[noise_idx] + cfg.mean[noise_idx];
			noise_idx += 1;
		}
		if (cfg.is_set & 0x02) {
			p = noise_.Sample() * cfg.sigma[noise_idx] + cfg.mean[noise_idx];
			noise_idx += 1;
		}
		if (cfg.is_set & 0x04) {
			y = noise_.Sample() * cfg.sigma[noise_idx] + cfg.mean[noise_idx];
		}
		tf2::Quaternion q_rot;
		q_rot.setRPY(r, p, y);
		q_rot.normalize();
		msg.quaternion = tf2::toMsg((q_rot * q_orig).normalize());
		return msg;
	}
};

} // namespace

std::unique_ptr<ISensorStream> MakeNativeStream(const mjModel *model, int sensor_id, std::string name,
                                                const SensorConfig &config, NoiseSource &noise, bool eval_mode)
{
	switch (model->sensor_type[sensor_id]) {
		case mjSENS_FRAMELINVEL:
		case mjSENS_FRAMELINACC:
		case mjSENS_FRAMEANGACC:
		case mjSENS_SUBTREECOM:
		case mjSENS_SUBTREELINVEL:
		case mjSENS_SUBTREEANGMOM:
		case mjSENS_ACCELEROMETER:
		case mjSENS_VELOCIMETER:
		case mjSENS_GYRO:
		case mjSENS_FORCE:
		case mjSENS_TORQUE:
		case mjSENS_MAGNETOMETER:
		case mjSENS_BALLANGVEL:
		case mjSENS_FRAMEXAXIS:
		case mjSENS_FRAMEYAXIS:
		case mjSENS_FRAMEZAXIS:
			return std::make_unique<Vector3Stream>(std::move(name), config, sensor_id, noise, eval_mode);
		case mjSENS_FRAMEPOS:
			return std::make_unique<PointStream>(std::move(name), config, sensor_id, noise, eval_mode);
		case mjSENS_TOUCH:
		case mjSENS_RANGEFINDER:
		case mjSENS_JOINTPOS:
		case mjSENS_JOINTVEL:
		case mjSENS_TENDONPOS:
		case mjSENS_TENDONVEL:
		case mjSENS_ACTUATORPOS:
		case mjSENS_ACTUATORVEL:
		case mjSENS_ACTUATORFRC:
		case mjSENS_JOINTACTFRC:
		case mjSENS_JOINTLIMITPOS:
		case mjSENS_JOINTLIMITVEL:
		case mjSENS_JOINTLIMITFRC:
		case mjSENS_TENDONLIMITPOS:
		case mjSENS_TENDONLIMITVEL:
		case mjSENS_TENDONLIMITFRC:
			return std::make_unique<ScalarStream>(std::move(name), config, sensor_id, noise, eval_mode);
		case mjSENS_BALLQUAT:
		case mjSENS_FRAMEQUAT:
			return std::make_unique<QuaternionStream>(std::move(name), config, sensor_id, noise, eval_mode);
		default:
			return nullptr;
	}
}

} // namespace mujoco_ros::sensors
