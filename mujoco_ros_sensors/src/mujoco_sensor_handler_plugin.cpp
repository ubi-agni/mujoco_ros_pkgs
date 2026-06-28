/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2025, Bielefeld University
 *  All rights reserved.
 */

/* Authors: David P. Leins */

#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.hpp>

#if MJR_ROS_VERSION == ROS_1
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#else
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>

namespace mujoco_ros::sensors {

static constexpr std::size_t SENSOR_PUBLISHER_QUEUE_SIZE = 1000;

#if MJR_ROS_VERSION == ROS_1
using PointStamped      = geometry_msgs::PointStamped;
using QuaternionStamped = geometry_msgs::QuaternionStamped;
using ScalarStamped     = mujoco_ros_msgs::ScalarStamped;
using Vector3Stamped    = geometry_msgs::Vector3Stamped;

template <typename MessageT>
ros::Publisher CreatePublisher(ros::NodeHandle &node, const std::string &topic, const std::string & /*ros2_type*/)
{
	return node.advertise<MessageT>(topic, SENSOR_PUBLISHER_QUEUE_SIZE, true);
}

template <typename MessageT>
void Publish(const ros::Publisher &publisher, const SensorConfig & /*config*/, const MessageT &message)
{
	publisher.publish(message);
}

ros::Time Now(ros::NodeHandle & /*node*/)
{
	return ros::Time::now();
}

#else
using PointStamped      = geometry_msgs::msg::PointStamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
using ScalarStamped     = mujoco_ros_msgs::msg::ScalarStamped;
using Vector3Stamped    = geometry_msgs::msg::Vector3Stamped;

template <typename MessageT>
typename rclcpp::Publisher<MessageT>::SharedPtr CreatePublisher(rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                                                const std::string &topic,
                                                                const std::string & /*ros2_type*/)
{
	return node->create_publisher<MessageT>(topic, rclcpp::QoS(SENSOR_PUBLISHER_QUEUE_SIZE));
}

template <typename MessageT>
void Publish(const std::any &publisher, const SensorConfig & /*config*/, const MessageT &message)
{
	std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(publisher)->publish(message);
}

rclcpp::Time Now(rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
{
	return node->now();
}

#endif

namespace {
void InitSensorStrings()
{
	SENSOR_STRING[mjSENS_TOUCH]          = "touch";
	SENSOR_STRING[mjSENS_ACCELEROMETER]  = "accelerometer";
	SENSOR_STRING[mjSENS_VELOCIMETER]    = "velocimeter";
	SENSOR_STRING[mjSENS_GYRO]           = "gyro";
	SENSOR_STRING[mjSENS_FORCE]          = "force";
	SENSOR_STRING[mjSENS_TORQUE]         = "torque";
	SENSOR_STRING[mjSENS_MAGNETOMETER]   = "magnetometer";
	SENSOR_STRING[mjSENS_RANGEFINDER]    = "rangefinder";
	SENSOR_STRING[mjSENS_CAMPROJECTION]  = "camprojection";
	SENSOR_STRING[mjSENS_JOINTPOS]       = "jointpos";
	SENSOR_STRING[mjSENS_JOINTVEL]       = "jointvel";
	SENSOR_STRING[mjSENS_TENDONPOS]      = "tendonpos";
	SENSOR_STRING[mjSENS_TENDONVEL]      = "tendonvel";
	SENSOR_STRING[mjSENS_ACTUATORPOS]    = "actuatorpos";
	SENSOR_STRING[mjSENS_ACTUATORVEL]    = "actuatorvel";
	SENSOR_STRING[mjSENS_ACTUATORFRC]    = "actuatorfrc";
	SENSOR_STRING[mjSENS_JOINTACTFRC]    = "jointactfrc";
	SENSOR_STRING[mjSENS_TENDONACTFRC]   = "tendonactfrc";
	SENSOR_STRING[mjSENS_BALLQUAT]       = "ballquat";
	SENSOR_STRING[mjSENS_BALLANGVEL]     = "ballangvel";
	SENSOR_STRING[mjSENS_JOINTLIMITPOS]  = "jointlimitpos";
	SENSOR_STRING[mjSENS_JOINTLIMITVEL]  = "jointlimitvel";
	SENSOR_STRING[mjSENS_JOINTLIMITFRC]  = "jointlimitfrc";
	SENSOR_STRING[mjSENS_TENDONLIMITPOS] = "tendonlimitpos";
	SENSOR_STRING[mjSENS_TENDONLIMITVEL] = "tendonlimitvel";
	SENSOR_STRING[mjSENS_TENDONLIMITFRC] = "tendonlimitfrc";
	SENSOR_STRING[mjSENS_FRAMEPOS]       = "framepos";
	SENSOR_STRING[mjSENS_FRAMEQUAT]      = "framequat";
	SENSOR_STRING[mjSENS_FRAMEXAXIS]     = "framexaxis";
	SENSOR_STRING[mjSENS_FRAMEYAXIS]     = "frameyaxis";
	SENSOR_STRING[mjSENS_FRAMEZAXIS]     = "framezaxis";
	SENSOR_STRING[mjSENS_FRAMELINVEL]    = "framelinvel";
	SENSOR_STRING[mjSENS_FRAMEANGVEL]    = "frameangvel";
	SENSOR_STRING[mjSENS_FRAMELINACC]    = "framelinacc";
	SENSOR_STRING[mjSENS_FRAMEANGACC]    = "frameangacc";
	SENSOR_STRING[mjSENS_SUBTREECOM]     = "subtreecom";
	SENSOR_STRING[mjSENS_SUBTREELINVEL]  = "subtreelinvel";
	SENSOR_STRING[mjSENS_SUBTREEANGMOM]  = "subtreeangmom";
	SENSOR_STRING[mjSENS_INSIDESITE]     = "insidesite";
	SENSOR_STRING[mjSENS_GEOMDIST]       = "geomdist";
	SENSOR_STRING[mjSENS_GEOMNORMAL]     = "geomnormal";
	SENSOR_STRING[mjSENS_GEOMFROMTO]     = "geomfromto";
	SENSOR_STRING[mjSENS_CONTACT]        = "contact";
	SENSOR_STRING[mjSENS_E_POTENTIAL]    = "e_potential";
	SENSOR_STRING[mjSENS_E_KINETIC]      = "e_kinetic";
	SENSOR_STRING[mjSENS_TACTILE]        = "tactile";
	SENSOR_STRING[mjSENS_PLUGIN]         = "plugin";
	SENSOR_STRING[mjSENS_USER]           = "user";
}
} // namespace

MujocoRosSensorsPlugin::~MujocoRosSensorsPlugin()
{
	sensor_map_.clear();
#if MJR_ROS_VERSION == ROS_1
	MJR_DEBUG_STREAM_NAMED("sensors", "Shutting down service " << register_noise_model_server_.getService());
	register_noise_model_server_.shutdown();
#else
	MJR_DEBUG_STREAM_NAMED("sensors", "Shutting down noise model server service");
#endif
}

bool MujocoRosSensorsPlugin::Load(const mjModel *model, mjData *data)
{
	MJR_INFO_NAMED("sensors", "Loading sensors plugin ...");
	if (env_ptr_->settings_.eval_mode) {
		MJR_WARN_NAMED("sensors", "Evaluation mode is active, ground truth topics won't be available!");
	} else {
		MJR_WARN_NAMED("sensors", "Train mode is active, ground truth topics will be available!");
	}

	InitSensorStrings();

#if MJR_ROS_VERSION == ROS_1
	std::string sensors_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		sensors_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	sensors_nh_ = ros::NodeHandle("/" + sensors_namespace);
#else
	sensors_nh_ = get_node();
#endif

	noise_dist = std::normal_distribution<double>(0.0, 1.0);
	InitSensors(model, data);
	MJR_INFO_NAMED("sensors", "All sensors initialized");

#if MJR_ROS_VERSION == ROS_1
	register_noise_model_server_ = sensors_nh_.advertiseService("sensors/register_noise_models",
	                                                            &MujocoRosSensorsPlugin::RegisterNoiseModelsCB, this);
#else
	register_noise_model_server_ = sensors_nh_->create_service<mujoco_ros_msgs::srv::RegisterSensorNoiseModels>(
	    "~/sensors/register_noise_models",
	    std::bind(&MujocoRosSensorsPlugin::RegisterNoiseModelsCB, this, std::placeholders::_1, std::placeholders::_2));
#endif

	return true;
}

#if MJR_ROS_VERSION == ROS_1
bool MujocoRosSensorsPlugin::RegisterNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
                                                   mujoco_ros_msgs::RegisterSensorNoiseModels::Response &resp)
#else
void MujocoRosSensorsPlugin::RegisterNoiseModelsCB(
    const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Request::SharedPtr &req,
    const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Response::SharedPtr &resp)
#endif
{
	if (env_ptr_->settings_.eval_mode) {
#if MJR_ROS_VERSION == ROS_1
		const auto &admin_hash = req.admin_hash;
#else
		const auto &admin_hash = req->admin_hash;
#endif
		if (env_ptr_->settings_.admin_hash != admin_hash) {
			MJR_ERROR_STREAM_NAMED("sensors", "Hash mismatch, no permission to change noise model!");
#if MJR_ROS_VERSION == ROS_1
			resp.success = false;
			return true;
#else
			resp->success = false;
			return;
#endif
		}
	}

#if MJR_ROS_VERSION == ROS_1
	auto &noise_models = req.noise_models;
#else
	auto &noise_models = req->noise_models;
#endif

	for (const auto &noise_model : noise_models) {
		MJR_WARN_STREAM_NAMED("sensors", "registering noise model for " << noise_model.sensor_name);
		int noise_idx = 0;

		const auto pos = sensor_map_.find(noise_model.sensor_name);
		if (pos == sensor_map_.end()) {
			MJR_WARN_STREAM_NAMED("sensors", "No sensor with name '"
			                                     << noise_model.sensor_name
			                                     << "' was registered on init. Can not apply noise model");
			continue;
		}

		const SensorConfigPtr &config = pos->second;

		if (noise_model.set_flag & 0x01) {
			config->mean[noise_idx]  = noise_model.mean[noise_idx];
			config->sigma[noise_idx] = noise_model.std[noise_idx];
			noise_idx += 1;
		}
		if (noise_model.set_flag & 0x02) {
			config->mean[noise_idx]  = noise_model.mean[noise_idx];
			config->sigma[noise_idx] = noise_model.std[noise_idx];
			noise_idx += 1;
		}
		if (noise_model.set_flag & 0x04) {
			config->mean[noise_idx]  = noise_model.mean[noise_idx];
			config->sigma[noise_idx] = noise_model.std[noise_idx];
		}

		config->is_set = config->is_set | noise_model.set_flag;
	}

#if MJR_ROS_VERSION == ROS_1
	resp.success = true;
	return true;
#else
	resp->success = true;
#endif
}

void MujocoRosSensorsPlugin::LastStageCallback(const mjModel *model, mjData *data)
{
	for (int n = 0; n < model->nsensor; n++) {
		const int adr     = model->sensor_adr[n];
		const int type    = model->sensor_type[n];
		const auto cutoff = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);

		if (!model->names[model->name_sensoradr[n]]) {
			continue;
		}

		std::string sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		auto config_it          = sensor_map_.find(sensor_name);
		if (config_it == sensor_map_.end()) {
			continue;
		}

		SensorConfig &config = *config_it->second;
		int noise_idx        = 0;
		double noise         = 0.0;

		switch (type) {
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
			case mjSENS_FRAMEZAXIS: {
				Vector3Stamped msg;
				msg.header.frame_id = config.frame_id;
				msg.header.stamp    = Now(sensors_nh_);

				for (int i = 0; i < 3; i++) {
					noise = 0.0;
					if (config.is_set & (1 << i)) {
						noise = noise_dist(rand_generator) * config.sigma[noise_idx] + config.mean[noise_idx];
						noise_idx += 1;
					}
					msg.vector.x = static_cast<float>((i == 0 ? data->sensordata[adr + i] + noise : msg.vector.x));
					msg.vector.y = static_cast<float>((i == 1 ? data->sensordata[adr + i] + noise : msg.vector.y));
					msg.vector.z = static_cast<float>((i == 2 ? data->sensordata[adr + i] + noise : msg.vector.z));
				}
				msg.vector.x /= cutoff;
				msg.vector.y /= cutoff;
				msg.vector.z /= cutoff;
				Publish(config.value_pub, config, msg);

				if (!env_ptr_->settings_.eval_mode) {
					msg.vector.x = static_cast<float>(data->sensordata[adr] / cutoff);
					msg.vector.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
					msg.vector.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);
					Publish(config.gt_pub, config, msg);
				}
				break;
			}
			case mjSENS_FRAMEPOS: {
				PointStamped msg;
				msg.header.frame_id = config.frame_id;
				msg.header.stamp    = Now(sensors_nh_);
				for (int i = 0; i < 3; i++) {
					noise = 0.0;
					if (config.is_set & (1 << i)) {
						noise = noise_dist(rand_generator) * config.sigma[noise_idx] + config.mean[noise_idx];
						noise_idx += 1;
					}
					msg.point.x = static_cast<float>((i == 0 ? data->sensordata[adr + i] + noise : msg.point.x));
					msg.point.y = static_cast<float>((i == 1 ? data->sensordata[adr + i] + noise : msg.point.y));
					msg.point.z = static_cast<float>((i == 2 ? data->sensordata[adr + i] + noise : msg.point.z));
				}
				msg.point.x /= cutoff;
				msg.point.y /= cutoff;
				msg.point.z /= cutoff;
				Publish(config.value_pub, config, msg);

				if (!env_ptr_->settings_.eval_mode) {
					msg.point.x = static_cast<float>(data->sensordata[adr] / cutoff);
					msg.point.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
					msg.point.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);
					Publish(config.gt_pub, config, msg);
				}
				break;
			}
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
			case mjSENS_TENDONLIMITFRC: {
				ScalarStamped msg;
				msg.header.frame_id = config.frame_id;
				msg.header.stamp    = Now(sensors_nh_);
				if (config.is_set & 0x01) {
					noise = noise_dist(rand_generator) * config.sigma[0] + config.mean[0];
				}
				msg.value = data->sensordata[adr] + noise / cutoff;
				Publish(config.value_pub, config, msg);

				if (!env_ptr_->settings_.eval_mode) {
					msg.value = data->sensordata[adr] / cutoff;
					Publish(config.gt_pub, config, msg);
				}
				break;
			}
			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT: {
				QuaternionStamped msg;
				msg.header.frame_id = config.frame_id;
				msg.header.stamp    = Now(sensors_nh_);
				msg.quaternion.w    = data->sensordata[adr] / cutoff;
				msg.quaternion.x    = data->sensordata[adr + 1] / cutoff;
				msg.quaternion.y    = data->sensordata[adr + 2] / cutoff;
				msg.quaternion.z    = data->sensordata[adr + 3] / cutoff;

				if (!env_ptr_->settings_.eval_mode) {
					Publish(config.gt_pub, config, msg);
				}

				if (config.is_set != 0) {
					tf2::Quaternion q_orig;
					tf2::Quaternion q_rot;
					tf2::fromMsg(msg.quaternion, q_orig);
					q_orig.normalize();

					double r = 0.0, p = 0.0, y = 0.0;
					if (config.is_set & 0x01) {
						r = noise_dist(rand_generator) * config.sigma[noise_idx] + config.mean[noise_idx];
						noise_idx += 1;
					}
					if (config.is_set & 0x02) {
						p = noise_dist(rand_generator) * config.sigma[noise_idx] + config.mean[noise_idx];
						noise_idx += 1;
					}
					if (config.is_set & 0x04) {
						y = noise_dist(rand_generator) * config.sigma[noise_idx] + config.mean[noise_idx];
					}
					q_rot.setRPY(r, p, y);
					q_rot.normalize();
					msg.quaternion = tf2::toMsg((q_rot * q_orig).normalize());
				}
				Publish(config.value_pub, config, msg);
				break;
			}
			default:
				MJR_ERROR_STREAM_NAMED("sensors",
				                       "Sensor publisher and frame_id defined but type is unsupported. This shouldn't "
				                       "happen! ("
				                           << sensor_name << " of type " << type << ")");
				break;
		}
	}
}

void MujocoRosSensorsPlugin::InitSensors(const mjModel *model, mjData * /*data*/)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		int site_id   = model->sensor_objid[n];
		int parent_id = model->site_bodyid[site_id];
		int type      = model->sensor_type[n];

		if (type == mjSENS_USER) {
			MJR_DEBUG_STREAM_NAMED("sensors", "Skipping USER sensor");
			continue;
		}

		site = mj_id2name(const_cast<mjModel *>(model), model->sensor_objtype[n], site_id);

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			MJR_WARN_STREAM_NAMED("sensors",
			                      "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site);
			continue;
		}

		bool global_frame = false;
		frame_id          = "world";
		SensorConfigPtr config;

		switch (type) {
			case mjSENS_FRAMEXAXIS:
			case mjSENS_FRAMEYAXIS:
			case mjSENS_FRAMEZAXIS:
			case mjSENS_FRAMELINVEL:
			case mjSENS_FRAMELINACC:
			case mjSENS_FRAMEANGACC: {
				int refid = model->sensor_refid[n];
				if (refid != -1) {
					int reftype = model->sensor_reftype[n];
					if (reftype == mjOBJ_SITE) {
						refid   = model->site_bodyid[refid];
						reftype = mjOBJ_BODY;
					}
					frame_id = mj_id2name(const_cast<mjModel *>(model), reftype, refid);
				}
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name, "geometry_msgs/msg/Vector3Stamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name + "_GT",
					                                                      "geometry_msgs/msg/Vector3Stamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				break;
			}
			case mjSENS_SUBTREECOM:
			case mjSENS_SUBTREELINVEL:
			case mjSENS_SUBTREEANGMOM:
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name, "geometry_msgs/msg/Vector3Stamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name + "_GT",
					                                                      "geometry_msgs/msg/Vector3Stamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
			case mjSENS_FRAMEPOS: {
				int refid = model->sensor_refid[n];
				if (refid != -1) {
					int reftype = model->sensor_reftype[n];
					if (reftype == mjOBJ_SITE) {
						refid   = model->site_bodyid[refid];
						reftype = mjOBJ_BODY;
					}
					frame_id = mj_id2name(const_cast<mjModel *>(model), reftype, refid);
				}
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<PointStamped>(sensors_nh_, sensor_name, "geometry_msgs/msg/PointStamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(
					    CreatePublisher<PointStamped>(sensors_nh_, sensor_name + "_GT", "geometry_msgs/msg/PointStamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
			}
			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT:
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<QuaternionStamped>(sensors_nh_, sensor_name, "geometry_msgs/msg/QuaternionStamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(CreatePublisher<QuaternionStamped>(sensors_nh_, sensor_name + "_GT",
					                                                         "geometry_msgs/msg/QuaternionStamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
		}

		if (global_frame || frame_id != "world") {
			MJR_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
			                                                       << frame_id << ") of type " << SENSOR_STRING[type]);
			continue;
		}

		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);
		MJR_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
		                                                       << frame_id << ") of type " << SENSOR_STRING[type]);

		switch (type) {
			case mjSENS_ACCELEROMETER:
			case mjSENS_VELOCIMETER:
			case mjSENS_GYRO:
			case mjSENS_FORCE:
			case mjSENS_TORQUE:
			case mjSENS_MAGNETOMETER:
			case mjSENS_BALLANGVEL:
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name, "geometry_msgs/msg/Vector3Stamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(CreatePublisher<Vector3Stamped>(sensors_nh_, sensor_name + "_GT",
					                                                      "geometry_msgs/msg/Vector3Stamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				break;
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
				config = std::make_unique<SensorConfig>(frame_id);
				config->RegisterPub(
				    CreatePublisher<ScalarStamped>(sensors_nh_, sensor_name, "mujoco_ros_msgs/msg/ScalarStamped"));
				if (!env_ptr_->settings_.eval_mode) {
					config->RegisterGTPub(CreatePublisher<ScalarStamped>(sensors_nh_, sensor_name + "_GT",
					                                                     "mujoco_ros_msgs/msg/ScalarStamped"));
				}
				sensor_map_[sensor_name] = std::move(config);
				break;
			default:
				MJR_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}
}

void MujocoRosSensorsPlugin::Reset() {}

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin)
