/**
 * Software License Agreement (BSD 3-Clause License)
 *
 *  Copyright (c) 2022, Bielefeld University
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
#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <mujoco_ros/mujoco_sim.h>

namespace mujoco_ros_sensors {

MujocoRosSensorsPlugin::~MujocoRosSensorsPlugin()
{
	// Kill deferred thread, if it is still blocking
	pthread_cancel(deferred_load_thread_.native_handle());
};

bool MujocoRosSensorsPlugin::load(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data)
{
	ROS_INFO_NAMED("sensors_plugin", "Loading sensors plugin ...");
	deferred_load_thread_ = boost::thread(boost::bind(&MujocoRosSensorsPlugin::initSensors, this, model, data));

	return true;
}

void MujocoRosSensorsPlugin::lastStageCallback(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data)
{
	if (!got_transform_)
		return;

	ros::Publisher pub;
	std::string frame_id, sensor_name;

	int adr, type;
	mjtNum cutoff;

	char tmp[mjMAXUITEXT];
	for (int n = 0; n < model->nsensor; n++) {
		adr    = model->sensor_adr[n];
		type   = model->sensor_type[n];
		cutoff = (model->sensor_cutoff[n] > 0 ? model->sensor_cutoff[n] : 1);

		if (model->names[model->name_sensoradr[n]]) {
			mju::strcat_arr(tmp, model->names + model->name_sensoradr[n]);
			sensor_name = std::string(tmp);
		} else {
			continue;
		}
		// reset tmp string
		tmp[0] = '\0';

		if (sensor_map_.find(sensor_name) == sensor_map_.end())
			continue;

		std::tie(pub, frame_id) = sensor_map_[sensor_name];

		switch (type) {
			case mjSENS_ACCELEROMETER:
			case mjSENS_FORCE: {
				geometry_msgs::Vector3Stamped msg;
				msg.header.frame_id = frame_id;
				msg.vector.x        = (float)(data->sensordata[adr] / cutoff);
				msg.vector.y        = (float)(data->sensordata[adr + 1] / cutoff);
				msg.vector.z        = (float)(data->sensordata[adr + 2] / cutoff);
				pub.publish(msg);
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
}

void MujocoRosSensorsPlugin::initSensors(MujocoSim::mjModelPtr model, MujocoSim::mjDataPtr data)
{
	// Wait for transforms to be available, then check if the frame ids exists
	tf2_ros::Buffer tfBuffer;
	ROS_INFO_NAMED("sensors", "sensors_plugin is waiting for /tf to be published ...");
	ros::topic::waitForMessage<tf2_msgs::TFMessage>("/tf", *node_handle_);

	ROS_INFO_NAMED("sensors", "/tf message received");

	char tmp[mjMAXUITEXT];
	std::string sensor_name;
	for (int n = 0; n < model->nsensor; n++) {
		int adr              = model->sensor_adr[n];
		int site_id          = model->sensor_objid[n];
		int parent_id        = model->site_bodyid[site_id];
		int type             = model->sensor_type[n];
		std::string site     = mj_id2name(model.get(), model->sensor_objtype[n], site_id);
		std::string frame_id = mj_id2name(model.get(), mjOBJ_BODY, parent_id);

		if (model->names[model->name_sensoradr[n]]) {
			mju::strcat_arr(tmp, model->names + model->name_sensoradr[n]);
			sensor_name = std::string(tmp);
		} else {
			ROS_WARN_STREAM_NAMED("sensors",
			                      "Sensor name resolution error. Skipping sensor of type " << type << " on site " << site);
			continue;
		}
		// reset tmp string
		tmp[0] = '\0';

		ROS_DEBUG_STREAM_NAMED("sensors", "Setting up sensor " << sensor_name << " on site " << site << " (frame_id: "
		                                                       << frame_id << ") of type " << SENSOR_STRING[type]);

		if (tf2_ros::Buffer()._frameExists(frame_id)) {
			ROS_WARN_STREAM_NAMED("sensors", "Cannot find frame with id '"
			                                     << frame_id << "' to publish sensor information from, skipping sensor.");
			continue;
		}

		std::string topic = frame_id + "/" + std::string(SENSOR_STRING[type]);

		switch (type) {
			case mjSENS_ACCELEROMETER:
			case mjSENS_FORCE:
				sensor_map_[sensor_name] =
				    std::pair(node_handle_->advertise<geometry_msgs::Vector3Stamped>(sensor_name, 10), frame_id);
				break;

			default:
				ROS_WARN_STREAM_NAMED("sensors", "Sensor of type '" << type << "' (" << sensor_name
				                                                    << ") is unknown! Cannot publish to ROS");
				break;
		}
	}

	got_transform_ = true;
}

void MujocoRosSensorsPlugin::reset()
{
	got_transform_ = false;
	sensor_map_.clear();
	deferred_load_thread_ = boost::thread(boost::bind(&MujocoRosSensorsPlugin::initSensors, this, model, data));
}

} // namespace mujoco_ros_sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros_sensors::MujocoRosSensorsPlugin, MujocoSim::MujocoPlugin)
