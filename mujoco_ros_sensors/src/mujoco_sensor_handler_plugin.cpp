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

#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mujoco_ros_msgs/CamProjectionStamped.h>
#include <mujoco_ros_msgs/GeomFromToStamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#else
#include <pluginlib/class_list_macros.hpp>

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mujoco_ros_msgs/msg/cam_projection_stamped.hpp>
#include <mujoco_ros_msgs/msg/geom_from_to_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <mujoco_ros/logging.hpp>
#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros_sensors/native_sensor_streams.hpp>

#include <mujoco_ros/util.hpp>

namespace mujoco_ros::sensors {

namespace {
bool IsGapSensorType(int type)
{
	switch (type) {
		case mjSENS_FRAMEANGVEL:
		case mjSENS_TENDONACTFRC:
		case mjSENS_E_POTENTIAL:
		case mjSENS_E_KINETIC:
		case mjSENS_INSIDESITE:
		case mjSENS_GEOMDIST:
		case mjSENS_GEOMNORMAL:
		case mjSENS_GEOMFROMTO:
		case mjSENS_CAMPROJECTION:
			return true;
		default:
			return false;
	}
}

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

MujocoRosSensorsPlugin::MujocoRosSensorsPlugin() = default;

MujocoRosSensorsPlugin::~MujocoRosSensorsPlugin()
{
	if (hub_registered_) {
		env_ptr_->sensors().UnregisterProvider(*this);
	}
	streams_.clear();
	compound_configs_.clear();
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
	model_ = model;
	data_  = data;

#if MJR_ROS_VERSION == ROS_1
	std::string sensors_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		sensors_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	sensors_nh_ = ros::NodeHandle("/" + sensors_namespace);
#else
	sensors_nh_ = get_node();
#endif

	InitSensors(model, data);
	MJR_INFO_NAMED("sensors", "All sensors initialized");

	if (!InitCompounds(model)) {
		return false;
	}
	// Only now, once claims are final, so a claimed part never gets a publisher.
	AdvertisePublicStreams();

	// A public name already taken by another provider (e.g. a laser) must fail the load loudly; a half-registered
	// plugin must not load and must not leave an uncaught throw behind.
	try {
		env_ptr_->sensors().RegisterProvider(*this);
		hub_registered_ = true;
	} catch (const std::exception &e) {
		MJR_ERROR_STREAM_NAMED("sensors", "Could not register sensor streams on the sensor hub: " << e.what());
		return false;
	}

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

		// A claimed part's config feeds its compound's value; noise there would be applied unseen.
		const auto stream_it = streams_.find(noise_model.sensor_name);
		if (stream_it != streams_.end() && !stream_it->second->is_public()) {
			MJR_WARN_STREAM_NAMED("sensors", "Sensor '" << noise_model.sensor_name
			                                            << "' is claimed by a compound sensor. Can not apply noise model");
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
	// New step: drop every cache first, claimed parts included, so a compound never composes a part's stale value
	// (streams_ is name-ordered, a part may sort after its compound) and publish and Python share one build per step.
	for (const auto &[name, stream] : streams_) {
		stream->InvalidateCache();
	}
	for (const auto &[name, stream] : streams_) {
		if (stream->is_public()) {
			stream->MaybePublish(model, data);
		}
	}
}

mujoco_ros::SensorStreamBase *MujocoRosSensorsPlugin::Find(const std::string &name) const
{
	const auto it = streams_.find(name);
	if (it == streams_.end() || !it->second->is_public()) {
		return nullptr;
	}
	return it->second.get();
}

std::any MujocoRosSensorsPlugin::ReadSensorValue(const std::string &name)
{
	auto *stream = static_cast<ISensorStream *>(Find(name));
	if (stream == nullptr) {
		throw std::out_of_range("no public sensor stream named '" + name + "'");
	}
	return stream->Value(model_, data_);
}

std::any MujocoRosSensorsPlugin::ReadSensorGroundTruth(const std::string &name)
{
	auto *stream = static_cast<ISensorStream *>(Find(name));
	if (stream == nullptr) {
		throw std::out_of_range("no public sensor stream named '" + name + "'");
	}
	return stream->GroundTruth(model_, data_);
}

std::vector<std::string> MujocoRosSensorsPlugin::PublicNames() const
{
	std::vector<std::string> names;
	for (const auto &[name, stream] : streams_) {
		if (stream->is_public()) {
			names.push_back(name);
		}
	}
	return names;
}

void MujocoRosSensorsPlugin::InitGapSensor(const std::string &sensor_name, const std::string &frame_id)
{
	sensor_map_[sensor_name] = std::make_unique<SensorConfig>(frame_id);
}

double MujocoRosSensorsPlugin::ReadUpdateRate(const std::string &sensor_name) const
{
#if MJR_ROS_VERSION == ROS_1
	// XmlRpcValue's conversion operators are non-const, so work on copies.
	XmlRpc::XmlRpcValue plugin_cfg = rosparam_config_;
	if (!plugin_cfg.hasMember(sensor_name)) {
		return 0.0;
	}
	XmlRpc::XmlRpcValue sensor_cfg = plugin_cfg[sensor_name];
	if (!sensor_cfg.hasMember("update_rate")) {
		return 0.0;
	}
	XmlRpc::XmlRpcValue value = sensor_cfg["update_rate"];
	if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		return static_cast<double>(value);
	}
	if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
		return static_cast<int>(value);
	}
	return 0.0;
#else
	const std::string param = "MujocoPlugins." + get_name() + "." + sensor_name + ".update_rate";
	if (!env_ptr_->has_parameter(param)) {
		return 0.0;
	}
	const auto value = env_ptr_->get_parameter(param);
	if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
		return static_cast<double>(value.as_int());
	}
	if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
		return value.as_double();
	}
	MJR_WARN_STREAM_NAMED("sensors", "Ignoring non-numeric update_rate for sensor " << sensor_name);
	return 0.0;
#endif
}

void MujocoRosSensorsPlugin::InitSensors(const mjModel *model, mjData * /*data*/)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		const int type = model->sensor_type[n];

		if (IsIgnoredSensorType(type)) {
			MJR_WARN_STREAM_NAMED("sensors", "Skipping sensor " << n << " of type " << type
			                                                    << ": no ROS message exists for this sensor type");
			continue;
		}

		if (!model->names[model->name_sensoradr[n]]) {
			MJR_WARN_STREAM_NAMED("sensors",
			                      "Sensor name resolution error. Skipping unnamed sensor " << n << " of type " << type);
			continue;
		}
		sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);

		// A mismatch would make the stream read into the next sensor's data or drop values.
		const int expected_dim = ExpectedNativeDim(type);
		if (expected_dim >= 0 && model->sensor_dim[n] != expected_dim) {
			MJR_ERROR_STREAM_NAMED("sensors", "Skipping sensor " << sensor_name << " (type " << type << "): dim "
			                                                     << model->sensor_dim[n] << " != expected "
			                                                     << expected_dim);
			continue;
		}

		if (IsGapSensorType(type)) {
			std::string error;
			if (!ResolveGapFrameId(model, n, frame_id, error)) {
				MJR_ERROR_STREAM_NAMED("sensors", "Skipping sensor " << sensor_name << " (type " << type << "): " << error);
				continue;
			}
			InitGapSensor(sensor_name, frame_id);
			MJR_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " (frame_id: " << frame_id << ")");
			continue;
		}

		const int site_id     = model->sensor_objid[n];
		const int parent_id   = model->site_bodyid[site_id];
		const char *site_cstr = mj_id2name(const_cast<mjModel *>(model), model->sensor_objtype[n], site_id);
		site                  = site_cstr ? site_cstr : "<unnamed>";

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
				config                   = std::make_unique<SensorConfig>(frame_id);
				sensor_map_[sensor_name] = std::move(config);
				break;
			}
			case mjSENS_SUBTREECOM:
			case mjSENS_SUBTREELINVEL:
			case mjSENS_SUBTREEANGMOM:
				config                   = std::make_unique<SensorConfig>(frame_id);
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
				config                   = std::make_unique<SensorConfig>(frame_id);
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
			}
			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT:
				config                   = std::make_unique<SensorConfig>(frame_id);
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
				config                   = std::make_unique<SensorConfig>(frame_id);
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
				config                   = std::make_unique<SensorConfig>(frame_id);
				sensor_map_[sensor_name] = std::move(config);
				break;
			default:
				MJR_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}

	for (int n = 0; n < model->nsensor; n++) {
		if (model->sensor_type[n] == mjSENS_USER || !model->names[model->name_sensoradr[n]]) {
			continue;
		}
		const std::string name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		const auto config_it   = sensor_map_.find(name);
		if (config_it == sensor_map_.end()) {
			continue;
		}
		config_it->second->update_rate = ReadUpdateRate(name);
		auto stream = MakeNativeStream(model, n, name, *config_it->second, noise_, env_ptr_->settings_.eval_mode);
		if (!stream) {
			MJR_ERROR_STREAM_NAMED("sensors", "Sensor " << name << " has a config but no stream (type "
			                                            << model->sensor_type[n] << ")");
			continue;
		}
		streams_[name] = std::move(stream);
	}
}

bool MujocoRosSensorsPlugin::ParseCompoundSpecs(std::vector<CompoundSpec> &specs, std::string &error) const
{
#if MJR_ROS_VERSION == ROS_1
	// XmlRpcValue's conversion operators are non-const, so work on copies.
	XmlRpc::XmlRpcValue plugin_cfg = rosparam_config_;
	if (!plugin_cfg.hasMember("compound_sensors")) {
		return true;
	}
	XmlRpc::XmlRpcValue list = plugin_cfg["compound_sensors"];
	if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		error = "compound_sensors must be a list";
		return false;
	}
	for (int i = 0; i < list.size(); i++) {
		XmlRpc::XmlRpcValue entry = list[i];
		if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			error = "compound_sensors entry " + std::to_string(i) + " must be a map";
			return false;
		}
		CompoundSpec spec;
		for (auto it = entry.begin(); it != entry.end(); ++it) {
			const std::string &key    = it->first;
			XmlRpc::XmlRpcValue value = it->second;
			if (key == "update_rate") {
				if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
					spec.update_rate = static_cast<double>(value);
				} else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
					spec.update_rate = static_cast<int>(value);
				} else {
					error = "compound_sensors entry " + std::to_string(i) + ": update_rate must be a number";
					return false;
				}
				continue;
			}
			if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
				error = "compound_sensors entry " + std::to_string(i) + ": '" + key + "' must be a string";
				return false;
			}
			const std::string text = static_cast<std::string>(value);
			if (key == "name") {
				spec.name = text;
			} else if (key == "type") {
				spec.type = text;
			} else {
				spec.roles[key] = text;
			}
		}
		specs.push_back(std::move(spec));
	}
	return true;
#else
	const std::string base = "MujocoPlugins." + get_name() + ".compound_sensors";
	if (!env_ptr_->has_parameter(base)) {
		return true;
	}
	const auto names_param = env_ptr_->get_parameter(base);
	if (names_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
		return true;
	}
	if (names_param.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
		error = "'" + base + "' must be a string array of compound names";
		return false;
	}
	for (const auto &name : names_param.as_string_array()) {
		const std::string prefix = base + "." + name;
		CompoundSpec spec;
		spec.name = name;
		for (const auto &param_name : env_ptr_->list_parameters({ prefix }, 2).names) {
			if (param_name.rfind(prefix + ".", 0) != 0) {
				continue;
			}
			const std::string key = param_name.substr(prefix.size() + 1);
			const auto value      = env_ptr_->get_parameter(param_name);
			if (key == "update_rate") {
				if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
					spec.update_rate = value.as_double();
				} else if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
					spec.update_rate = static_cast<double>(value.as_int());
				} else {
					error = "'" + param_name + "' must be a number";
					return false;
				}
				continue;
			}
			if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
				error = "'" + param_name + "' must be a string";
				return false;
			}
			if (key == "type") {
				spec.type = value.as_string();
			} else {
				spec.roles[key] = value.as_string();
			}
		}
		specs.push_back(std::move(spec));
	}
	return true;
#endif
}

bool MujocoRosSensorsPlugin::InitCompounds(const mjModel *model)
{
	compounds_.clear();
	std::vector<CompoundSpec> specs;
	std::string error;
	if (!ParseCompoundSpecs(specs, error)) {
		MJR_ERROR_STREAM_NAMED("sensors", "Invalid compound_sensors config: " << error);
		return false;
	}

	std::map<std::string, std::string> part_frame_ids;
	// Only sensors with a live stream can be claimed; a config without a stream has nothing to demote.
	for (const auto &[name, stream] : streams_) {
		part_frame_ids[name] = stream->config().frame_id;
	}
	if (!ValidateCompounds(model, specs, part_frame_ids, compounds_, error)) {
		MJR_ERROR_STREAM_NAMED("sensors", "Invalid compound_sensors config: " << error);
		return false;
	}
	ApplyCompounds();
	return true;
}

void MujocoRosSensorsPlugin::ApplyCompounds()
{
	for (const auto &desc : compounds_) {
		if (!HasCompoundStream(desc.kind)) {
			// Demoting would drop the parts without any stream taking their place.
			MJR_WARN_STREAM_NAMED("sensors", "Compound sensor '" << desc.name
			                                                     << "' has no stream implementation for this type yet; "
			                                                        "its parts stay public and it is not published");
			continue;
		}

		auto config         = std::make_unique<SensorConfig>(desc.frame_id);
		config->update_rate = desc.update_rate;

		std::map<std::string, ISensorStream *> parts;
		for (const auto &[role, sensor] : desc.roles) {
			parts[role] = streams_.at(sensor).get();
		}
		auto stream = MakeCompoundStream(desc, *config, parts, env_ptr_->settings_.eval_mode);

		// Nothing is advertised yet (see AdvertisePublicStreams), so a claimed part never reaches the topic graph.
		for (const auto &[role, sensor] : desc.roles) {
			streams_.at(sensor)->Demote();
		}
		compound_configs_[desc.name] = std::move(config);
		streams_[desc.name]          = std::move(stream);
		MJR_DEBUG_STREAM_NAMED("sensors",
		                       "Compound sensor " << desc.name << " claims " << desc.roles.size() << " sensors");
	}
}

void MujocoRosSensorsPlugin::AdvertisePublicStreams()
{
	const bool gt = !env_ptr_->settings_.eval_mode;
	for (const auto &[name, stream] : streams_) {
		if (!stream->is_public()) {
			continue;
		}
		auto native  = sensor_map_.find(name);
		auto &config = native != sensor_map_.end() ? *native->second : *compound_configs_.at(name);
		stream->Advertise(sensors_nh_, config, gt);
	}
}

void MujocoRosSensorsPlugin::Reset()
{
	for (const auto &[name, stream] : streams_) {
		stream->InvalidateCache();
	}
}

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin)
