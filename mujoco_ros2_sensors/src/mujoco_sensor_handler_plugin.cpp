/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022-2024, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Authors: David P. Leins */

#include <mujoco_ros2_sensors/mujoco_sensor_handler_plugin.h>



// #include <geometry_msgs/PointStamped.h>
// #include <geometry_msgs/QuaternionStamped.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <mujoco_ros_msgs/ScalarStamped.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mujoco_ros/mujoco_env.hpp>

// TODO:
// x All the .publish to -> publish, 
// x creating publishers, 
// message creation, 
// x node handle-related functions, 
// ros time
namespace mujoco_ros::sensors {

MujocoRos2SensorsPlugin::~MujocoRos2SensorsPlugin()
{
	sensor_map_.clear();
	RCLCPP_DEBUG_STREAM(getLogger(), "Shutting down noise model server service");
	// register_noise_model_server_->shutdown();
}

bool MujocoRos2SensorsPlugin::Load(const mjModel *model, mjData *data)
{
	RCLCPP_INFO(getLogger(), "Loading sensors plugin ...");
	if (env_ptr_->settings_.eval_mode) {
		RCLCPP_WARN(getLogger(), "Evaluation mode is active, ground truth topics won't be available!");
	} else {
		RCLCPP_WARN(getLogger(), "Train mode is active, ground truth topics will be available!");
	}

	SENSOR_STRING[mjSENS_TOUCH]          = "touch";
	SENSOR_STRING[mjSENS_ACCELEROMETER]  = "accelerometer";
	SENSOR_STRING[mjSENS_VELOCIMETER]    = "velocimeter";
	SENSOR_STRING[mjSENS_GYRO]           = "gyro";
	SENSOR_STRING[mjSENS_FORCE]          = "force";
	SENSOR_STRING[mjSENS_TORQUE]         = "torque";
	SENSOR_STRING[mjSENS_MAGNETOMETER]   = "magnetometer";
	SENSOR_STRING[mjSENS_RANGEFINDER]    = "rangefinder";
	SENSOR_STRING[mjSENS_JOINTPOS]       = "jointpos";
	SENSOR_STRING[mjSENS_JOINTVEL]       = "jointvel";
	SENSOR_STRING[mjSENS_TENDONPOS]      = "tendonpos";
	SENSOR_STRING[mjSENS_TENDONVEL]      = "tendonvel";
	SENSOR_STRING[mjSENS_ACTUATORPOS]    = "actuatorpos";
	SENSOR_STRING[mjSENS_ACTUATORVEL]    = "actuatorvel";
	SENSOR_STRING[mjSENS_ACTUATORFRC]    = "actuatorfrc";
	SENSOR_STRING[mjSENS_BALLQUAT]       = "ballquat";
	SENSOR_STRING[mjSENS_BALLANGVEL]     = "ballangvel";
	SENSOR_STRING[mjSENS_JOINTACTFRC]    = "jointactfrc";
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

	// TODO
	// std::string sensors_namespace;
	// if (rosparam_config_.hasMember("namespace")) {
	// 	sensors_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	// }
	// sensors_nh_ = ros::NodeHandle("/" + sensors_namespace);
	sensors_nh_ = get_node();

	noise_dist = std::normal_distribution<double>(0.0, 1.0);
	configureLidarMap(model, data);
	initSensors(model, data);
	RCLCPP_INFO(sensors_nh_->get_logger(), "All sensors initialized");

	register_noise_model_server_ = sensors_nh_->create_service<mujoco_ros_msgs::srv::RegisterSensorNoiseModels>("~/sensors/register_noise_models",
	                                                            std::bind(&MujocoRos2SensorsPlugin::registerNoiseModelsCB, 
																		  this, 
																		  std::placeholders::_1, 
																		  std::placeholders::_2));

	return true;
}

void MujocoRos2SensorsPlugin::registerNoiseModelsCB(const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Request::SharedPtr &req,
                                                    const mujoco_ros_msgs::srv::RegisterSensorNoiseModels::Response::SharedPtr &resp)
{
	if (env_ptr_->settings_.eval_mode) {
		RCLCPP_DEBUG(rclcpp::get_logger("mujoco"), "Evaluation mode is active. Checking hash validity");
		if (env_ptr_->settings_.admin_hash != req->admin_hash) {
			RCLCPP_ERROR(rclcpp::get_logger("mujoco"), "Hash mismatch, no permission to change noise model!");
			resp->success = false;
			// return true;
		}
		RCLCPP_DEBUG(rclcpp::get_logger("mujoco"), "Hash valid, request authorized.");
	}

	int noise_idx;
	for (auto &noise_model : req->noise_models) {
		RCLCPP_WARN_STREAM(getLogger(), "registering noise model for " << noise_model.sensor_name);
		noise_idx = 0;

		const std::map<std::string, SensorConfigPtr>::const_iterator &pos = sensor_map_.find(noise_model.sensor_name);
		if (pos == sensor_map_.end()) {
			RCLCPP_WARN_STREAM(getLogger(), "No sensor with name '"
			                                     << noise_model.sensor_name
			                                     << "' was registered on init. Can not apply noise model");
			continue;
		}

		const SensorConfigPtr &config = pos->second;
		RCLCPP_DEBUG_STREAM_EXPRESSION(getLogger(), config->is_set > 0, "Overriding current noise model with newly provided");

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

	resp->success = true;

	// return true;
}

void MujocoRos2SensorsPlugin::LastStageCallback(const mjModel *model, mjData *data)
{
	std::string sensor_name;

	int adr, type, noise_idx;
	mjtNum cutoff;
	double noise = 0.0;

	for (int n = 0; n < model->nsensor; n++) {
		adr       = model->sensor_adr[n];
		type      = model->sensor_type[n];
		cutoff    = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
		noise_idx = 0;

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		if (sensor_map_.find(sensor_name) == sensor_map_.end())
			continue;

		SensorConfigPtr &config = sensor_map_[sensor_name];

		switch (type) {
			{
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
					geometry_msgs::msg::Vector3Stamped msg;
					msg.header.frame_id = config->frame_id;
					msg.header.stamp    = sensors_nh_->now();

					// No noise configured
					if (config->is_set == 0) {
						msg.vector.x = static_cast<float>(data->sensordata[adr] / cutoff);
						msg.vector.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
						msg.vector.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);

						config->value_pub->publish(config->serializeMessage(msg));

						if (!env_ptr_->settings_.eval_mode) {
							config->gt_pub->publish(config->serializeMessage(msg));
						}
					} else { // Noise at least in one dim
						if (config->is_set & 0x01) {
							// shift and scale standard normal to desired distribution
							noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
							noise_idx += 1;
						} else {
							noise = 0;
						}
						msg.vector.x = static_cast<float>(data->sensordata[adr] + noise / cutoff);

						if (config->is_set & 0x02) {
							// shift and scale standard normal to desired distribution
							noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
							noise_idx += 1;
						} else {
							noise = 0;
						}
						msg.vector.y = (float)(data->sensordata[adr + 1] + noise / cutoff);

						if (config->is_set & 0x04) {
							// shift and scale standard normal to desired distribution
							noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						} else {
							noise = 0;
						}
						msg.vector.z = (float)(data->sensordata[adr + 2] + noise / cutoff);

						config->value_pub->publish(config->serializeMessage(msg));

						if (!env_ptr_->settings_.eval_mode) {
							msg.vector.x = static_cast<float>(data->sensordata[adr] / cutoff);
							msg.vector.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
							msg.vector.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);

							config->gt_pub->publish(config->serializeMessage(msg));
						}
					}
					break;
			}

			case mjSENS_FRAMEPOS: {
				geometry_msgs::msg::PointStamped msg;
				msg.header.frame_id = config->frame_id;
				msg.header.stamp    = sensors_nh_->now();

				// No noise configured
				if (config->is_set == 0) {
					msg.point.x = static_cast<float>(data->sensordata[adr] / cutoff);
					msg.point.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
					msg.point.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);

					config->value_pub->publish(config->serializeMessage(msg));

					if (!env_ptr_->settings_.eval_mode) {
						config->gt_pub->publish(config->serializeMessage(msg));
					}
				} else { // Noise at least in one dim
					if (config->is_set & 0x01) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						noise_idx += 1;
					} else {
						noise = 0;
					}
					msg.point.x = static_cast<float>(data->sensordata[adr] + noise / cutoff);

					if (config->is_set & 0x02) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						noise_idx += 1;
					} else {
						noise = 0;
					}
					msg.point.y = static_cast<float>(data->sensordata[adr + 1] + noise / cutoff);

					if (config->is_set & 0x04) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
					} else {
						noise = 0;
					}
					msg.point.z = static_cast<float>(data->sensordata[adr + 2] + noise / cutoff);

					config->value_pub->publish(config->serializeMessage(msg));

					if (!env_ptr_->settings_.eval_mode) {
						msg.point.x = static_cast<float>(data->sensordata[adr] / cutoff);
						msg.point.y = static_cast<float>(data->sensordata[adr + 1] / cutoff);
						msg.point.z = static_cast<float>(data->sensordata[adr + 2] / cutoff);

						config->gt_pub->publish(config->serializeMessage(msg));
					}
				}
				break;
			}

				{
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
						mujoco_ros_msgs::msg::ScalarStamped msg;
						msg.header.frame_id = config->frame_id;
						msg.header.stamp    = sensors_nh_->now();
						// No noise configured
						if (config->is_set == 0) {
							msg.value = static_cast<float>(data->sensordata[adr] / cutoff);

							config->value_pub->publish(config->serializeMessage(msg));

							if (!env_ptr_->settings_.eval_mode) {
								config->gt_pub->publish(config->serializeMessage(msg));
							}
						} else { // Noise set
							// shift and scale standard normal to desired distribution
							noise     = noise_dist(rand_generator) * config->sigma[0] + config->mean[0];
							msg.value = static_cast<float>(data->sensordata[adr] + noise / cutoff);

							config->value_pub->publish(config->serializeMessage(msg));

							if (!env_ptr_->settings_.eval_mode) {
								msg.value = static_cast<float>(data->sensordata[adr] / cutoff);

								config->gt_pub->publish(config->serializeMessage(msg));
							}
						}
						break;
				}

			case mjSENS_BALLQUAT: {
				case mjSENS_FRAMEQUAT:
					geometry_msgs::msg::QuaternionStamped msg;
					tf2::Quaternion q_orig, q_rot;
					msg.header.frame_id = config->frame_id;
					msg.header.stamp    = sensors_nh_->now();

					msg.quaternion.w = static_cast<float>(data->sensordata[adr] / cutoff);
					msg.quaternion.x = static_cast<float>(data->sensordata[adr + 1] / cutoff);
					msg.quaternion.y = static_cast<float>(data->sensordata[adr + 2] / cutoff);
					msg.quaternion.z = static_cast<float>(data->sensordata[adr + 3] / cutoff);

					if (!env_ptr_->settings_.eval_mode) {
						config->gt_pub->publish(config->serializeMessage(msg));
					}

					if (config->is_set == 0) {
						config->value_pub->publish(config->serializeMessage(msg));
					} else {
						tf2::fromMsg(msg.quaternion, q_orig);
						q_orig.normalize();

						double r, p, y;

						if (config->is_set & 0x01) {
							// shift and scale standard normal to desired distribution
							r = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
							noise_idx += 1;
						} else {
							r = 0;
						}
						if (config->is_set & 0x02) {
							// shift and scale standard normal to desired distribution
							p = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
							noise_idx += 1;
						} else {
							p = 0;
						}
						if (config->is_set & 0x04) {
							// shift and scale standard normal to desired distribution
							y = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						} else {
							y = 0;
						}

						q_rot.setRPY(r, p, y);
						q_rot.normalize();

						msg.quaternion = tf2::toMsg((q_rot * q_orig).normalize());
						config->value_pub->publish(config->serializeMessage(msg));
					}
					break;
			}

			default:
				RCLCPP_ERROR_STREAM(
				    getLogger(),
				    "Sensor publisher and frame_id defined but type can't be serialized. This shouldn't happen! ("
				        << sensor_name << " of type " << type << ")");
				break;
		}
	}
	publishLidarData(model, data);
}

void MujocoRos2SensorsPlugin::initSensors(const mjModel *model, mjData *data)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		int site_id   = model->sensor_objid[n];
		int parent_id = model->site_bodyid[site_id];
		int type      = model->sensor_type[n];

		// Skip user sensors because handling is unknown and should be done in extra plugin
		if (type == mjSENS_USER) {
			RCLCPP_INFO_STREAM(getLogger(), "Skipping USER sensor");
			continue;
		}

		site = mj_id2name(const_cast<mjModel *>(model), model->sensor_objtype[n], site_id);

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			RCLCPP_WARN_STREAM(getLogger(),
			                      "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site);
			continue;
		}

		// Global frame sensors
		bool global_frame = false;
		frame_id          = "world";
		SensorConfigPtr config;
		switch (type) {
			{
				case mjSENS_FRAMEXAXIS:
				case mjSENS_FRAMEYAXIS:
				case mjSENS_FRAMEZAXIS:
				case mjSENS_FRAMELINVEL:
				case mjSENS_FRAMELINACC:
				case mjSENS_FRAMEANGACC:
					int refid = model->sensor_refid[n];
					if (refid != -1) {
						int reftype = model->sensor_reftype[n];
						if (reftype == mjOBJ_SITE) {
							refid   = model->site_bodyid[refid];
							reftype = mjOBJ_BODY;
						}
						frame_id = mj_id2name(const_cast<mjModel *>(model), reftype, refid);
						RCLCPP_DEBUG_STREAM(getLogger(), "Sensor has relative frame with id " << refid << " and type "
						                                                                       << reftype << " and ref_frame "
						                                                                       << frame_id);
					}
					config = std::make_unique<SensorConfig>(frame_id);
					config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "geometry_msgs/msg/Vector3Stamped", 1);
					if (!env_ptr_->settings_.eval_mode) {
						config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "geometry_msgs/msg/Vector3Stamped", 1);
					}
					sensor_map_[sensor_name] = std::move(config);
					break;
			}
			case mjSENS_SUBTREECOM:
			case mjSENS_SUBTREELINVEL:
			case mjSENS_SUBTREEANGMOM:
				config = std::make_unique<SensorConfig>(frame_id);
				config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "geometry_msgs/msg/Vector3Stamped", 1);
				if (!env_ptr_->settings_.eval_mode) {
					config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "geometry_msgs/msg/Vector3Stamped", 1);
				}
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
				{
					case mjSENS_FRAMEPOS:
						int refid = model->sensor_refid[n];
						if (refid != -1) {
							int reftype = model->sensor_reftype[n];
							if (reftype == mjOBJ_SITE) {
								refid   = model->site_bodyid[refid];
								reftype = mjOBJ_BODY;
							}
							frame_id = mj_id2name(const_cast<mjModel *>(model), reftype, refid);
							RCLCPP_DEBUG_STREAM(getLogger(), "Sensor has relative frame with id "
							                                      << refid << " and type " << reftype << " and ref_frame "
							                                      << frame_id);
						}
						config = std::make_unique<SensorConfig>(frame_id);
						config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "geometry_msgs/msg/PointStamped", 1);
						if (!env_ptr_->settings_.eval_mode) {
							config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "geometry_msgs/msg/PointStamped", 1);
						}
						sensor_map_[sensor_name] = std::move(config);
						global_frame             = true;
						break;
				}

			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT:
				config = std::make_unique<SensorConfig>(frame_id);
				config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "geometry_msgs/msg/QuaternionStamped", 1);
				if (!env_ptr_->settings_.eval_mode) {
					config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "geometry_msgs/msg/QuaternionStamped", 1);
				}
				sensor_map_[sensor_name] = std::move(config);
				global_frame             = true;
				break;
		}

		// Check if sensor is in global frame and already setup
		if (global_frame || frame_id != "world") {
			RCLCPP_DEBUG_STREAM(getLogger(), "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
			                                                       << frame_id << ") of type " << SENSOR_STRING[type]);
			continue;
		}

		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);
		RCLCPP_DEBUG_STREAM(getLogger(), "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
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
				// create_publisher<sensor_msgs/msg/JointState>("~/joint_states", 1);
				config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "geometry_msgs/msg/Vector3Stamped", 1);
				if (!env_ptr_->settings_.eval_mode) {
					config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "geometry_msgs/msg/Vector3Stamped", 1);
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
				config->value_pub = sensors_nh_->create_generic_publisher(sensor_name, "mujoco_ros_msgs/msg/ScalarStamped", 1);
				if (!env_ptr_->settings_.eval_mode) {
					config->gt_pub = sensors_nh_->create_generic_publisher(sensor_name + "_GT", "mujoco_ros_msgs/msg/ScalarStamped", 1);
				}
				sensor_map_[sensor_name] = std::move(config);
				break;

			default:
				RCLCPP_WARN_STREAM(getLogger(), "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}
}

void MujocoRos2SensorsPlugin::configureLidarMap(const mjModel *model, mjData *data){
	auto lidar_names = sensors_nh_->get_parameter("lidars").as_string_array();
	for (const auto &lidar_name : lidar_names) {
		RCLCPP_INFO(getLogger(), "Configuring lidar %s", lidar_name.c_str());
		// Do we really need to manually declare params like this?
		sensors_nh_->declare_parameter<double>(lidar_name+".max",-1);
		sensors_nh_->declare_parameter<double>(lidar_name+".min",-1);
		sensors_nh_->declare_parameter<double>(lidar_name+".angle",0);
		sensors_nh_->declare_parameter<int>(lidar_name+".rf_count",0);
		
		LidarConfigPtr lidar = std::make_unique<LidarConfig>();
		if(sensors_nh_->has_parameter(lidar_name + ".max")){
			lidar->max_ = sensors_nh_->get_parameter(lidar_name+".max").as_double();
		}
		if(sensors_nh_->has_parameter(lidar_name + ".min")){
			lidar->min_ = sensors_nh_->get_parameter(lidar_name+".min").as_double();
		}
		if(sensors_nh_->has_parameter(lidar_name + ".angle")){
			lidar->angle_ = sensors_nh_->get_parameter(lidar_name+".angle").as_double();
		}
		if(sensors_nh_->has_parameter(lidar_name + ".rf_count")){
			lidar->rf_count_ = sensors_nh_->get_parameter(lidar_name+".rf_count").as_int();
		}
		// Make the lidar only valid if all the parameters have been set correctly,
		// since the lidar data calculation depends on it
		lidar->valid_ = (lidar->max_ > 0 &&  
						 lidar->min_ > 0 && 
						 lidar->max_ > lidar->min_ &&  // max should be bigger than min
						 lidar->rf_count_ > 0 &&  // there must be at least 1 rangefinder
						 lidar->angle_ > 0); // angle between each rangefinder must be bigger than 0.
		
		// now calculate by how much the lidar object needs to be rotated at each tick, in order to cover a full circle given the number of rangefinders.
		lidar->angle_to_rotate_ = lidar->angle_ * (lidar->rf_count_);
		lidar->accumulated_angle_ = 0;
		// Lidars are populated using repeat on a site.
		// so actually, the lidar name in the config should be its SITE, not the SENSOR.
		// Sensors populated using repeat will have no name.
		std::string frame_id, site_name;
		int site_id;
		for(int i = 0; i < lidar->rf_count_; i++){
			// validate that the number of RF matches the number of sites
			site_name = lidar_name + std::to_string(i);
			site_id = mj_name2id(const_cast<mjModel *>(model), mjOBJ_SITE, site_name.c_str());

			if(site_id == -1){
				lidar->valid_ = false;
				break;
			}

			for(int n = 0; n < model->nsensor; n++){
				// check that the sites all have 1 sensor of rangefinder type attached to it
				if(model->sensor_objtype[n] == mjOBJ_SITE && model->sensor_objid[n] == site_id && model->sensor_type[n] == mjSENS_RANGEFINDER){
					lidar->sensor_ids_.push_back(n);
					break;
				}
			}
		}

		// ensure that the number of rangefinder sensors in the vector container is the same as rf_count
		if(lidar->sensor_ids_.size() != (size_t)lidar->rf_count_){
			RCLCPP_WARN(getLogger(), "Number of rangefinder sensors in %s is not %d! Found %ld.", lidar_name.c_str(), lidar->rf_count_, lidar->sensor_ids_.size());
			lidar->valid_ = false;
		}
		
		// if any of the conditions fail, we move on
		if(!lidar->valid_){
			RCLCPP_WARN(getLogger(), "Lidar %s is not valid! Skipping it.", lidar_name.c_str());
			lidar_map_[lidar_name] = std::move(lidar); // still move it, to have record for what lidar names were given.
			continue;
		}
		
		// now we've validated what we can, move on with setting up other stuff
		lidar->body_id_ = model->site_bodyid[site_id];
		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, model->site_bodyid[site_id]);
		lidar->msg_ = sensor_msgs::msg::LaserScan();
		lidar->msg_.header.frame_id = frame_id;
		lidar->msg_.range_max = lidar->max_;
		lidar->msg_.range_min = lidar->min_;
		lidar->msg_.angle_min = 0;
		lidar->msg_.angle_max = M_PI*2;
		
		RCLCPP_INFO(getLogger(), "Lidar %s config:\nRanges: [%f %f], Angle min/max: [%f %f], increment: %f, rf_count: %d", 
								lidar_name.c_str(), lidar->min_, lidar->max_, lidar->msg_.angle_min, lidar->msg_.angle_max, lidar->angle_, lidar->rf_count_);
		lidar->msg_.angle_increment = lidar->angle_;
		lidar->msg_.scan_time = (M_PI*2/lidar->angle_to_rotate_) / (1.0/model->opt.timestep); 
		lidar->msg_.time_increment = model->opt.timestep / lidar->rf_count_;

		RCLCPP_INFO(getLogger(), "Creating publishers for lidar %s in frame %s", lidar_name.c_str(), frame_id.c_str());
		SensorConfigPtr config;
		config = std::make_unique<SensorConfig>(frame_id);
		config->value_pub = sensors_nh_->create_generic_publisher(lidar_name, "sensor_msgs/msg/LaserScan", 1);
		if (!env_ptr_->settings_.eval_mode) {
			config->gt_pub = sensors_nh_->create_generic_publisher(lidar_name + "_GT", "sensor_msgs/msg/LaserScan", 1);
		}
		sensor_map_[lidar_name] = std::move(config);
		lidar_map_[lidar_name] = std::move(lidar);
	}

};


void MujocoRos2SensorsPlugin::publishLidarData(const mjModel* model, mjData *data){

	for (const auto &lidar_config : lidar_map_) {

		if(!lidar_config.second->valid_){
			// don't bother with invalid lidars
			continue;
		}

		// read the data. Since we are reading then adding to accumulated angle, this should ensure that
		// sensor data from rangefinder that exceeds the 360 degree are not written to the message.
		for(int i = 0; i < lidar_config.second->rf_count_; i++){
			if(lidar_config.second->accumulated_angle_ >= M_PI*2){
				// if the accumulated angle exceeds 360, stop adding sensor data
				break;
			}
			lidar_config.second->msg_.ranges.push_back(data->sensordata[lidar_config.second->sensor_ids_[i]]);
			lidar_config.second->accumulated_angle_ += lidar_config.second->angle_;
			// RCLCPP_INFO(getLogger(), "Current angle: %f", lidar_config.second->accumulated_angle_);
		}
		// if the accumulated angle >= 360 degrees, reset it to 0, and publish the message with timestamp
		if(lidar_config.second->accumulated_angle_ >= M_PI*2){
			lidar_config.second->msg_.header.stamp = sensors_nh_->now();
			sensor_map_[lidar_config.first]->value_pub->publish(sensor_map_[lidar_config.first]->serializeMessage(lidar_config.second->msg_));
			data->qpos[model->jnt_qposadr[model->body_jntadr[lidar_config.second->body_id_]]] = 0;
			lidar_config.second->accumulated_angle_ = 0.0;
			lidar_config.second->msg_.ranges.clear(); // clear out the message at the beginning
		}
		else{
			// rotate the body, i.e. the joint on which the body is attached to.
			// RCLCPP_INFO(getLogger(), "Rotating by %f", lidar_config.second->accumulated_angle_);
			data->qpos[model->jnt_qposadr[model->body_jntadr[lidar_config.second->body_id_]]] = lidar_config.second->accumulated_angle_;
		}

	};
}
// Nothing to do on reset
void MujocoRos2SensorsPlugin::Reset(){};

} // namespace mujoco_ros::sensors
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRos2SensorsPlugin, mujoco_ros::MujocoPlugin)
