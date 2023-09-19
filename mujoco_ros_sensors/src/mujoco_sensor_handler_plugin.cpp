/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2023, Bielefeld University
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

#include <mujoco_ros_sensors/mujoco_sensor_handler_plugin.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mujoco_ros/mujoco_env.h>

namespace mujoco_ros::sensors {

MujocoRosSensorsPlugin::~MujocoRosSensorsPlugin()
{
	sensor_map_.clear();
	ROS_DEBUG_STREAM_NAMED("sensors", "Shutting down service " << register_noise_model_server_.getService());
	register_noise_model_server_.shutdown();
}

bool MujocoRosSensorsPlugin::load(mujoco_ros::mjModelPtr model, mujoco_ros::mjDataPtr data)
{
	ROS_INFO_NAMED("sensors", "Loading sensors plugin ...");
	if (env_ptr_->settings_.eval_mode) {
		ROS_WARN_NAMED("sensors", "Evalutaion mode is active, ground truth topics won't be available!");
	} else {
		ROS_WARN_NAMED("sensors", "Train mode is active, ground truth topics will be available!");
	}

	std::string sensors_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		sensors_namespace = (std::string)rosparam_config_["namespace"];
	}
	sensors_nh_.reset(new ros::NodeHandle("/" + sensors_namespace));

	noise_dist = std::normal_distribution<double>(0.0, 1.0);
	initSensors(model, data);
	ROS_INFO_NAMED("sensors", "All sensors initialized");

	register_noise_model_server_ = sensors_nh_->advertiseService("sensors/register_noise_models",
	                                                             &MujocoRosSensorsPlugin::registerNoiseModelsCB, this);

	return true;
}

bool MujocoRosSensorsPlugin::registerNoiseModelsCB(mujoco_ros_msgs::RegisterSensorNoiseModels::Request &req,
                                                   mujoco_ros_msgs::RegisterSensorNoiseModels::Response &resp)
{
	if (env_ptr_->settings_.eval_mode) {
		ROS_DEBUG_NAMED("mujoco", "Evaluation mode is active. Checking hash validity");
		if (env_ptr_->settings_.admin_hash != req.admin_hash) {
			ROS_ERROR_NAMED("mujoco", "Hash mismatch, no permission to change noise model!");
			resp.success = false;
			return true;
		}
		ROS_DEBUG_NAMED("mujoco", "Hash valid, request authorized.");
	}

	SensorConfigPtr config;
	int noise_idx;
	for (const mujoco_ros_msgs::SensorNoiseModel &noise_model : req.noise_models) {
		ROS_WARN_STREAM_NAMED("sensors", "registering noise model for " << noise_model.sensor_name);
		noise_idx = 0;

		const std::map<std::string, SensorConfigPtr>::const_iterator &pos = sensor_map_.find(noise_model.sensor_name);
		if (pos == sensor_map_.end()) {
			ROS_WARN_STREAM_NAMED("sensors", "No sensor with name '"
			                                     << noise_model.sensor_name
			                                     << "' was registered on init. Can not apply noise model");
			continue;
		}

		config = pos->second;

		ROS_DEBUG_STREAM_COND_NAMED(config->is_set > 0, "sensors", "Overriding current noise model with newly provided");

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

	resp.success = true;

	return true;
}

void MujocoRosSensorsPlugin::lastStageCallback(mujoco_ros::mjModelPtr model, mujoco_ros::mjDataPtr data)
{
	std::string sensor_name;

	int adr, type, noise_idx;
	mjtNum cutoff;
	SensorConfigPtr config;
	double noise = 0.0;

	for (int n = 0; n < model->nsensor; n++) {
		adr       = model->sensor_adr[n];
		type      = model->sensor_type[n];
		cutoff    = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
		noise_idx = 0;

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(model.get(), mjOBJ_SENSOR, n);
		} else {
			continue;
		}

		if (sensor_map_.find(sensor_name) == sensor_map_.end())
			continue;

		config = sensor_map_[sensor_name];

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
					geometry_msgs::Vector3Stamped msg;
					msg.header.frame_id = config->frame_id;
					msg.header.stamp    = ros::Time::now();

					// No noise configured
					if (config->is_set == 0) {
						msg.vector.x = (float)(data->sensordata[adr] / cutoff);
						msg.vector.y = (float)(data->sensordata[adr + 1] / cutoff);
						msg.vector.z = (float)(data->sensordata[adr + 2] / cutoff);

						config->value_pub.publish(msg);

						if (!env_ptr_->settings_.eval_mode) {
							config->gt_pub.publish(msg);
						}
					} else { // Noise at least in one dim
						if (config->is_set & 0x01) {
							// shift and scale standard normal to desired distribution
							noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
							noise_idx += 1;
						} else {
							noise = 0;
						}
						msg.vector.x = (float)(data->sensordata[adr] + noise / cutoff);

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

						config->value_pub.publish(msg);

						if (!env_ptr_->settings_.eval_mode) {
							msg.vector.x = (float)(data->sensordata[adr] / cutoff);
							msg.vector.y = (float)(data->sensordata[adr + 1] / cutoff);
							msg.vector.z = (float)(data->sensordata[adr + 2] / cutoff);

							config->gt_pub.publish(msg);
						}
					}
					break;
			}

			case mjSENS_FRAMEPOS: {
				geometry_msgs::PointStamped msg;
				msg.header.frame_id = config->frame_id;
				msg.header.stamp    = ros::Time::now();

				// No noise configured
				if (config->is_set == 0) {
					msg.point.x = (float)(data->sensordata[adr] / cutoff);
					msg.point.y = (float)(data->sensordata[adr + 1] / cutoff);
					msg.point.z = (float)(data->sensordata[adr + 2] / cutoff);

					config->value_pub.publish(msg);

					if (!env_ptr_->settings_.eval_mode) {
						config->gt_pub.publish(msg);
					}
				} else { // Noise at least in one dim
					if (config->is_set & 0x01) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						noise_idx += 1;
					} else {
						noise = 0;
					}
					msg.point.x = (float)(data->sensordata[adr] + noise / cutoff);

					if (config->is_set & 0x02) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
						noise_idx += 1;
					} else {
						noise = 0;
					}
					msg.point.y = (float)(data->sensordata[adr + 1] + noise / cutoff);

					if (config->is_set & 0x04) {
						// shift and scale standard normal to desired distribution
						noise = noise_dist(rand_generator) * config->sigma[noise_idx] + config->mean[noise_idx];
					} else {
						noise = 0;
					}
					msg.point.z = (float)(data->sensordata[adr + 2] + noise / cutoff);

					config->value_pub.publish(msg);

					if (!env_ptr_->settings_.eval_mode) {
						msg.point.x = (float)(data->sensordata[adr] / cutoff);
						msg.point.y = (float)(data->sensordata[adr + 1] / cutoff);
						msg.point.z = (float)(data->sensordata[adr + 2] / cutoff);

						config->gt_pub.publish(msg);
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
					case mjSENS_JOINTLIMITPOS:
					case mjSENS_JOINTLIMITVEL:
					case mjSENS_JOINTLIMITFRC:
					case mjSENS_TENDONLIMITPOS:
					case mjSENS_TENDONLIMITVEL:
					case mjSENS_TENDONLIMITFRC:
						mujoco_ros_msgs::ScalarStamped msg;
						msg.header.frame_id = config->frame_id;
						msg.header.stamp    = ros::Time::now();

						// No noise configured
						if (config->is_set == 0) {
							msg.value = (float)(data->sensordata[adr] / cutoff);

							config->value_pub.publish(msg);

							if (!env_ptr_->settings_.eval_mode) {
								config->gt_pub.publish(msg);
							}
						} else { // Noise set
							// shift and scale standard normal to desired distribution
							noise     = noise_dist(rand_generator) * config->sigma[0] + config->mean[0];
							msg.value = (float)(data->sensordata[adr] + noise / cutoff);

							config->value_pub.publish(msg);

							if (!env_ptr_->settings_.eval_mode) {
								msg.value = (float)(data->sensordata[adr] / cutoff);

								config->gt_pub.publish(msg);
							}
						}
						break;
				}

			case mjSENS_BALLQUAT: {
				case mjSENS_FRAMEQUAT:
					geometry_msgs::QuaternionStamped msg;
					tf2::Quaternion q_orig, q_rot;
					msg.header.frame_id = config->frame_id;
					msg.header.stamp    = ros::Time::now();

					msg.quaternion.w = (float)(data->sensordata[adr] / cutoff);
					msg.quaternion.x = (float)(data->sensordata[adr + 1] / cutoff);
					msg.quaternion.y = (float)(data->sensordata[adr + 2] / cutoff);
					msg.quaternion.z = (float)(data->sensordata[adr + 3] / cutoff);

					if (!env_ptr_->settings_.eval_mode) {
						config->gt_pub.publish(msg);
					}

					if (config->is_set == 0) {
						config->value_pub.publish(msg);
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
						config->value_pub.publish(msg);
					}
					break;
			}

			default:
				ROS_ERROR_STREAM_NAMED(
				    "sensors",
				    "Sensor publisher and frame_id defined but type can't be serialized. This shouldn't happen! ("
				        << sensor_name << " of type " << type << ")");
				break;
		}
	}

	for (std::string wrench_sensor_name : wrench_sensor_names_) {
		auto wrench_sensor_config = wrench_sensor_map_[wrench_sensor_name];
		auto force_sensor_config  = wrench_sensor_config->force_config;
		auto torque_sensor_config = wrench_sensor_config->torque_config;

		geometry_msgs::WrenchStamped msg;
		msg.header.frame_id = force_sensor_config->frame_id;
		msg.header.stamp    = ros::Time::now();

		int id    = wrench_sensor_config->force_sensor_id;
		adr       = model->sensor_adr[id];
		type      = model->sensor_type[id];
		cutoff    = (model->sensor_cutoff[id] > 0 ? model->sensor_cutoff[id] : 1);
		noise_idx = 0;

		// No noise configured
		if (force_sensor_config->is_set == 0) {
			msg.wrench.force.x = (float)(data->sensordata[adr] / cutoff);
			msg.wrench.force.y = (float)(data->sensordata[adr + 1] / cutoff);
			msg.wrench.force.z = (float)(data->sensordata[adr + 2] / cutoff);
		} else { // Noise at least in one dim
			if (force_sensor_config->is_set & 0x01) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * force_sensor_config->sigma[noise_idx] +
				        force_sensor_config->mean[noise_idx];
				noise_idx += 1;
			} else {
				noise = 0;
			}
			msg.wrench.force.x = (float)(data->sensordata[adr] + noise / cutoff);

			if (force_sensor_config->is_set & 0x02) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * force_sensor_config->sigma[noise_idx] +
				        force_sensor_config->mean[noise_idx];
				noise_idx += 1;
			} else {
				noise = 0;
			}
			msg.wrench.force.y = (float)(data->sensordata[adr + 1] + noise / cutoff);

			if (force_sensor_config->is_set & 0x04) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * force_sensor_config->sigma[noise_idx] +
				        force_sensor_config->mean[noise_idx];
			} else {
				noise = 0;
			}
			msg.wrench.force.z = (float)(data->sensordata[adr + 2] + noise / cutoff);
		}

		id        = wrench_sensor_config->torque_sensor_id;
		adr       = model->sensor_adr[id];
		type      = model->sensor_type[id];
		cutoff    = (model->sensor_cutoff[id] > 0 ? model->sensor_cutoff[id] : 1);
		noise_idx = 0;

		// No noise configured
		if (torque_sensor_config->is_set == 0) {
			msg.wrench.torque.x = (float)(data->sensordata[adr] / cutoff);
			msg.wrench.torque.y = (float)(data->sensordata[adr + 1] / cutoff);
			msg.wrench.torque.z = (float)(data->sensordata[adr + 2] / cutoff);
		} else { // Noise at least in one dim
			if (torque_sensor_config->is_set & 0x01) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * torque_sensor_config->sigma[noise_idx] +
				        torque_sensor_config->mean[noise_idx];
				noise_idx += 1;
			} else {
				noise = 0;
			}
			msg.wrench.torque.x = (float)(data->sensordata[adr] + noise / cutoff);

			if (torque_sensor_config->is_set & 0x02) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * torque_sensor_config->sigma[noise_idx] +
				        torque_sensor_config->mean[noise_idx];
				noise_idx += 1;
			} else {
				noise = 0;
			}
			msg.wrench.torque.y = (float)(data->sensordata[adr + 1] + noise / cutoff);

			if (torque_sensor_config->is_set & 0x04) {
				// shift and scale standard normal to desired distribution
				noise = noise_dist(rand_generator) * torque_sensor_config->sigma[noise_idx] +
				        torque_sensor_config->mean[noise_idx];
			} else {
				noise = 0;
			}
			msg.wrench.torque.z = (float)(data->sensordata[adr + 2] + noise / cutoff);
		}

		wrench_sensor_config->value_pub.publish(msg);

		if (!env_ptr_->settings_.eval_mode) {
			wrench_sensor_config->gt_pub.publish(msg);
		}
	}
}

void MujocoRosSensorsPlugin::initSensors(mujoco_ros::mjModelPtr model, mujoco_ros::mjDataPtr data)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		int adr       = model->sensor_adr[n];
		int site_id   = model->sensor_objid[n];
		int parent_id = model->site_bodyid[site_id];
		int type      = model->sensor_type[n];

		site = mj_id2name(model.get(), model->sensor_objtype[n], site_id);

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(model.get(), mjOBJ_SENSOR, n);
		} else {
			ROS_WARN_STREAM_NAMED("sensors",
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
						frame_id = mj_id2name(model.get(), reftype, refid);
						ROS_DEBUG_STREAM_NAMED("sensors", "Sensor has relative frame with id " << refid << " and type "
						                                                                       << reftype << " and ref_frame "
						                                                                       << frame_id);
					}
					config.reset(new SensorConfig(frame_id));
					config->registerPub(sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name, 1, true));
					if (!env_ptr_->settings_.eval_mode) {
						config->registerGTPub(
						    sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name + "_GT", 1, true));
					}
					sensor_map_[sensor_name] = config;
					break;
			}
			case mjSENS_SUBTREECOM:
			case mjSENS_SUBTREELINVEL:
			case mjSENS_SUBTREEANGMOM:
				config.reset(new SensorConfig(frame_id));
				config->registerPub(sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name, 1, true));
				if (!env_ptr_->settings_.eval_mode) {
					config->registerGTPub(
					    sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name + "_GT", 1, true));
				}
				sensor_map_[sensor_name] = config;
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
							frame_id = mj_id2name(model.get(), reftype, refid);
							ROS_DEBUG_STREAM_NAMED("sensors", "Sensor has relative frame with id "
							                                      << refid << " and type " << reftype << " and ref_frame "
							                                      << frame_id);
						}
						config.reset(new SensorConfig(frame_id));
						config->registerPub(sensors_nh_->advertise<geometry_msgs::PointStamped>(sensor_name, 1, true));
						if (!env_ptr_->settings_.eval_mode) {
							config->registerGTPub(
							    sensors_nh_->advertise<geometry_msgs::PointStamped>(sensor_name + "_GT", 1, true));
						}
						sensor_map_[sensor_name] = config;
						global_frame             = true;
						break;
				}

			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT:
				config.reset(new SensorConfig(frame_id));
				config->registerPub(sensors_nh_->advertise<geometry_msgs::QuaternionStamped>(sensor_name, 1, true));
				if (!env_ptr_->settings_.eval_mode) {
					config->registerGTPub(
					    sensors_nh_->advertise<geometry_msgs::QuaternionStamped>(sensor_name + "_GT", 1, true));
				}
				sensor_map_[sensor_name] = config;
				global_frame             = true;
				break;
		}

		// Check if sensor is in global frame and already setup
		if (global_frame || frame_id != "world") {
			ROS_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
			                                                       << frame_id << ") of type " << SENSOR_STRING[type]);
			continue;
		}

		frame_id = mj_id2name(model.get(), mjOBJ_BODY, parent_id);
		ROS_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
		                                                       << frame_id << ") of type " << SENSOR_STRING[type]);

		switch (type) {
			case mjSENS_ACCELEROMETER:
			case mjSENS_VELOCIMETER:
			case mjSENS_GYRO:
			case mjSENS_FORCE:
			case mjSENS_TORQUE:
			case mjSENS_MAGNETOMETER:
			case mjSENS_BALLANGVEL:
				config.reset(new SensorConfig(frame_id));
				config->registerPub(sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name, 1, true));
				if (!env_ptr_->settings_.eval_mode) {
					config->registerGTPub(
					    sensors_nh_->advertise<geometry_msgs::Vector3Stamped>(sensor_name + "_GT", 1, true));
				}
				sensor_map_[sensor_name] = config;
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
			case mjSENS_JOINTLIMITPOS:
			case mjSENS_JOINTLIMITVEL:
			case mjSENS_JOINTLIMITFRC:
			case mjSENS_TENDONLIMITPOS:
			case mjSENS_TENDONLIMITVEL:
			case mjSENS_TENDONLIMITFRC:
				config.reset(new SensorConfig(frame_id));
				config->registerPub(sensors_nh_->advertise<mujoco_ros_msgs::ScalarStamped>(sensor_name, 1, true));
				if (!env_ptr_->settings_.eval_mode) {
					config->registerGTPub(
					    sensors_nh_->advertise<mujoco_ros_msgs::ScalarStamped>(sensor_name + "_GT", 1, true));
				}
				sensor_map_[sensor_name] = config;
				break;

			default:
				ROS_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}

	for (int n = 0; n < model->ntuple; ++n) {
		std::string tuple_name;
		if (model->names[model->name_tupleadr[n]])
			tuple_name = mj_id2name(model.get(), mjOBJ_TUPLE, n);
		else {
			ROS_WARN_STREAM_NAMED("sensors", "Tuple name resolution error. Skipping");
			continue;
		}

		if (tuple_name != "wrench")
			continue;

		int adr  = model->tuple_adr[n];
		int size = model->tuple_size[n];
		WrenchSensorConfigPtr config;

		for (int i = 0; i < size; ++i) {
			std::string wrench_sensor_name =
			    mj_id2name(model.get(), model->tuple_objtype[adr + i], model->tuple_objid[adr + i]);

			if (mjOBJ_TUPLE != model->tuple_objtype[adr + i]) {
				ROS_WARN_STREAM_NAMED("sensors", "Wrench sensor '" << wrench_sensor_name << "' is not a tuple. Skipping");
				continue;
			}

			int id   = mj_name2id(model.get(), mjOBJ_TUPLE, wrench_sensor_name.c_str());
			int size = model->tuple_size[id];

			if (size != 2) {
				ROS_WARN_STREAM_NAMED("sensors", "Wrench sensor '" << wrench_sensor_name << "' is not a 2-tuple. Skipping");
				continue;
			}

			int adr = model->tuple_adr[id];

			if (model->tuple_objtype[adr] != mjOBJ_SENSOR) {
				ROS_WARN_STREAM_NAMED("sensors", "The first element of wrench sensor '" << wrench_sensor_name
				                                                                        << "' is not a sensor. Skipping");
				continue;
			}
			if (model->tuple_objtype[adr + 1] != mjOBJ_SENSOR) {
				ROS_WARN_STREAM_NAMED("sensors", "The second element of wrench sensor '" << wrench_sensor_name
				                                                                         << "' is not a sensor. Skipping");
				continue;
			}

			int force_sensor_id  = model->tuple_objid[adr];
			int torque_sensor_id = model->tuple_objid[adr + 1];

			if (model->sensor_type[force_sensor_id] != mjSENS_FORCE) {
				ROS_WARN_STREAM_NAMED("sensors", "The first element of wrench sensor '"
				                                     << wrench_sensor_name << "' is not a force sensor. Skipping");
				continue;
			}
			if (model->sensor_type[torque_sensor_id] != mjSENS_TORQUE) {
				ROS_WARN_STREAM_NAMED("sensors", "The second element of wrench sensor '"
				                                     << wrench_sensor_name << "' is not a force sensor. Skipping");
				continue;
			}

			std::string force_sensor_name  = mj_id2name(model.get(), mjOBJ_SENSOR, force_sensor_id);
			std::string torque_sensor_name = mj_id2name(model.get(), mjOBJ_SENSOR, torque_sensor_id);

			// ROS_INFO_STREAM_NAMED("sensors", "Wrench sensor '" << name << "' from '" << force_sensor_name << "' and
			// '"
			//                                                    << torque_sensor_name << "'");

			config.reset(new WrenchSensorConfig(force_sensor_id, sensor_map_[force_sensor_name], torque_sensor_id,
			                                    sensor_map_[torque_sensor_name]));

			if (config->force_config->frame_id != config->torque_config->frame_id) {
				ROS_ERROR_STREAM_NAMED("sensors", "The force sensor " << force_sensor_name << " and the torque sensor "
				                                                      << torque_sensor_name
				                                                      << " does not measure in the same frame");
				continue;
			}

			config->registerPub(sensors_nh_->advertise<geometry_msgs::WrenchStamped>(wrench_sensor_name, 1, true));
			if (!env_ptr_->settings_.eval_mode) {
				config->registerGTPub(
				    sensors_nh_->advertise<geometry_msgs::WrenchStamped>(wrench_sensor_name + "_GT", 1, true));
			}

			wrench_sensor_names_.push_back(wrench_sensor_name);
			wrench_sensor_map_[wrench_sensor_name] = config;
		}
	}

	// for (std::string name : wrench_sensor_names_) {
	// 	WrenchSensorConfigPtr config;

	// 	int id   = mj_name2id(model.get(), mjOBJ_TUPLE, name.c_str());
	// 	int size = model->tuple_size[id];

	// 	if (size != 2) {
	// 		ROS_WARN_STREAM_NAMED("sensors", "Wrench sensor '" << name << "' is not a 2-tuple. Skipping");
	// 		continue;
	// 	}

	// 	int adr = model->tuple_adr[id];

	// 	if (model->tuple_objtype[adr] != mjOBJ_SENSOR) {
	// 		ROS_WARN_STREAM_NAMED("sensors",
	// 		                      "The first element of wrench sensor '" << name << "' is not a sensor. Skipping");
	// 		continue;
	// 	}
	// 	if (model->tuple_objtype[adr + 1] != mjOBJ_SENSOR) {
	// 		ROS_WARN_STREAM_NAMED("sensors",
	// 		                      "The second element of wrench sensor '" << name << "' is not a sensor. Skipping");
	// 		continue;
	// 	}

	// 	int force_sensor_id  = model->tuple_objid[adr];
	// 	int torque_sensor_id = model->tuple_objid[adr + 1];

	// 	if (model->sensor_type[force_sensor_id] != mjSENS_FORCE) {
	// 		ROS_WARN_STREAM_NAMED("sensors",
	// 		                      "The first element of wrench sensor '" << name << "' is not a force sensor.
	// Skipping"); 		continue;
	// 	}
	// 	if (model->sensor_type[torque_sensor_id] != mjSENS_TORQUE) {
	// 		ROS_WARN_STREAM_NAMED("sensors",
	// 		                      "The second element of wrench sensor '" << name << "' is not a force sensor.
	// Skipping"); 		continue;
	// 	}

	// 	std::string force_sensor_name  = mj_id2name(model.get(), mjOBJ_SENSOR, force_sensor_id);
	// 	std::string torque_sensor_name = mj_id2name(model.get(), mjOBJ_SENSOR, torque_sensor_id);

	// 	// ROS_INFO_STREAM_NAMED("sensors", "Wrench sensor '" << name << "' from '" << force_sensor_name << "' and
	// '"
	// 	//                                                    << torque_sensor_name << "'");

	// 	config.reset(new WrenchSensorConfig(sensor_map_[force_sensor_name], sensor_map_[torque_sensor_name]));
	// 	config->registerPub(sensors_nh_->advertise<geometry_msgs::WrenchStamped>(name, 1, true));
	// 	if (!env_ptr_->settings_.eval_mode) {
	// 		config->registerGTPub(sensors_nh_->advertise<geometry_msgs::WrenchStamped>(name + "_GT", 1, true));
	// 	}

	// 	wrench_sensor_map_[name] = config;
	// }
}

// Nothing to do on reset
void MujocoRosSensorsPlugin::reset(){};

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin)
