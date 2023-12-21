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
#include <mujoco_ros_sensors/serialization.h>

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mujoco_ros_msgs/ScalarStamped.h>

#include <mujoco_ros/mujoco_env.h>

#include <stdexcept>

namespace mujoco_ros::sensors {

void RosSensorInterfaceBase::setNoise(const mjtNum *mu_noise, const mjtNum *sigma_noise, uint8_t noise_set)
{
	uint noise_count = noise_set - ((noise_set >> 1) & 033333333333) - ((noise_set >> 2) & 011111111111);
	noise_count      = ((noise_count + (noise_count >> 3)) & 030707070707) % 63;
	if (noise_count > dofs_) {
		throw std::runtime_error("Too many noise parameters provided for sensor " + sensor_name_);
	}

	int noise_idx = 0;
	if (noise_set & 0x01) {
		mu_noise_[noise_idx]    = mu_noise[noise_idx];
		sigma_noise_[noise_idx] = sigma_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x02) {
		mu_noise_[noise_idx]    = mu_noise[noise_idx];
		sigma_noise_[noise_idx] = sigma_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x04) {
		mu_noise_[noise_idx]    = mu_noise[noise_idx];
		sigma_noise_[noise_idx] = sigma_noise[noise_idx];
	}

	noise_set_ = noise_set_ | noise_set;
}

MujocoRosSensorsPlugin::~MujocoRosSensorsPlugin()
{
	for (auto &sensor : sensor_map_) {
		delete sensor.second;
	}
	sensor_map_.clear();
	ROS_DEBUG_STREAM_NAMED("sensors", "Shutting down service " << register_noise_model_server_.getService());
	register_noise_model_server_.shutdown();
}

bool MujocoRosSensorsPlugin::load(const mjModel *model, mjData *data)
{
	ROS_INFO_NAMED("sensors", "Loading sensors plugin ...");
	if (env_ptr_->settings_.eval_mode) {
		ROS_WARN_NAMED("sensors", "Evaluation mode is active, ground truth topics won't be available!");
	} else {
		ROS_WARN_NAMED("sensors", "Train mode is active, ground truth topics will be available!");
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

	std::string sensors_namespace;
	if (rosparam_config_.hasMember("namespace")) {
		sensors_namespace = static_cast<std::string>(rosparam_config_["namespace"]);
	}
	sensors_nh_ = ros::NodeHandle("/" + sensors_namespace);

	noise_dist_ = std::normal_distribution<double>(0.0, 1.0);
	initSensors(model, data);
	ROS_INFO_NAMED("sensors", "All sensors initialized");

	register_noise_model_server_ = sensors_nh_.advertiseService("sensors/register_noise_models",
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

	resp.success = true;
	for (const mujoco_ros_msgs::SensorNoiseModel &noise_model : req.noise_models) {
		ROS_WARN_STREAM_NAMED("sensors", "registering noise model for " << noise_model.sensor_name);

		const std::map<std::string, RosSensorInterfaceBase *>::const_iterator &pos =
		    sensor_map_.find(noise_model.sensor_name);
		if (pos == sensor_map_.end()) {
			ROS_WARN_STREAM_NAMED("sensors", "No sensor with name '"
			                                     << noise_model.sensor_name
			                                     << "' was registered on init. Can not apply noise model");
			resp.success = false;
			continue;
		}

		RosSensorInterfaceBase *sensor = pos->second;
		try {
			sensor->setNoise(noise_model.mean.data(), noise_model.std.data(), noise_model.set_flag);
		} catch (const std::runtime_error &e) {
			ROS_ERROR_STREAM_NAMED("sensors", "Error while setting noise model for sensor " << noise_model.sensor_name
			                                                                                << ": " << e.what());
			resp.success = false;
			return true;
		}
	}

	return true;
}

void MujocoRosSensorsPlugin::lastStageCallback(const mjModel *model, mjData *data)
{
	std::string sensor_name;

	std::map<std::string, RosSensorInterfaceBase *>::iterator pos = sensor_map_.begin();
	while (pos != sensor_map_.end()) {
		RosSensorInterfaceBase *sensor = pos->second;
		sensor->publish(!env_ptr_->settings_.eval_mode, data, noise_dist_, rand_generator_);
		pos++;
	}
}

void MujocoRosSensorsPlugin::initSensors(const mjModel *model, mjData *data)
{
	std::string sensor_name, site, frame_id;
	for (int n = 0; n < model->nsensor; n++) {
		int site_id   = model->sensor_objid[n];
		int parent_id = model->site_bodyid[site_id];
		int type      = model->sensor_type[n];
		mjtNum cutoff = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);
		int adr       = model->sensor_adr[n];

		site = mj_id2name(const_cast<mjModel *>(model), model->sensor_objtype[n], site_id);

		if (model->names[model->name_sensoradr[n]]) {
			sensor_name = mj_id2name(const_cast<mjModel *>(model), mjOBJ_SENSOR, n);
		} else {
			ROS_WARN_STREAM_NAMED("sensors",
			                      "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site);
			continue;
		}

		bool global_frame = false;
		frame_id          = "world";
		switch (type) {
			// Global or relative frame 3 DoF sensors
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
						ROS_DEBUG_STREAM_NAMED("sensors", "Sensor has relative frame with id " << refid << " and type "
						                                                                       << reftype << " and ref_frame "
						                                                                       << frame_id);
					}
					sensor_map_[sensor_name] = new RosSensorInterface<geometry_msgs::Vector3Stamped>(
					    frame_id, sensor_name, 3, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
					break;
			}
			// Global frame 3 DoF sensors
			case mjSENS_SUBTREECOM:
			case mjSENS_SUBTREELINVEL:
			case mjSENS_SUBTREEANGMOM:
				sensor_map_[sensor_name] = new RosSensorInterface<geometry_msgs::Vector3Stamped>(
				    frame_id, sensor_name, 3, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
				global_frame = true;
				break;
				// Global or local frame position sensors (1 DoF)
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
							ROS_DEBUG_STREAM_NAMED("sensors", "Sensor has relative frame with id "
							                                      << refid << " and type " << reftype << " and ref_frame "
							                                      << frame_id);
						}
						sensor_map_[sensor_name] = new RosSensorInterface<geometry_msgs::PointStamped>(
						    frame_id, sensor_name, 3, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
						global_frame = true;
						break;
				}

			// Global frame quaternion sensors (4 DoF)
			case mjSENS_BALLQUAT:
			case mjSENS_FRAMEQUAT:
				sensor_map_[sensor_name] = new RosSensorInterface<geometry_msgs::QuaternionStamped>(
				    frame_id, sensor_name, 3, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
				global_frame = true;
				break;
		}

		// If global_frame was explicitly set to true or frame_id was overridden by relative frame
		// the sensor is already registered and we can skip checking the remaining types
		if (global_frame || frame_id != "world") {
			ROS_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
			                                                       << frame_id << ") of type " << SENSOR_STRING[type]);
			continue;
		}

		// The remaining sensors all measure in the site's parent body frame
		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, parent_id);
		ROS_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
		                                                       << frame_id << ") of type " << SENSOR_STRING[type]);

		switch (type) {
			// 3D-vector type sensors
			case mjSENS_ACCELEROMETER:
			case mjSENS_VELOCIMETER:
			case mjSENS_GYRO:
			case mjSENS_FORCE:
			case mjSENS_TORQUE:
			case mjSENS_MAGNETOMETER:
			case mjSENS_BALLANGVEL:
				sensor_map_[sensor_name] = new RosSensorInterface<geometry_msgs::Vector3Stamped>(
				    frame_id, sensor_name, 3, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
				break;

			// Scalar type sensors
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
				sensor_map_[sensor_name] = new RosSensorInterface<mujoco_ros_msgs::ScalarStamped>(
				    frame_id, sensor_name, 1, adr, cutoff, env_ptr_->settings_.eval_mode, &sensors_nh_);
				break;

			default:
				ROS_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}
}

// Nothing to do on reset
void MujocoRosSensorsPlugin::reset(){};

} // namespace mujoco_ros::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRosSensorsPlugin, mujoco_ros::MujocoPlugin)
