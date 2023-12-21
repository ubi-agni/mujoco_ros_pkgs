/*********************************************************************
 * Software License Agreement (BSD License)
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
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
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
 *********************************************************************/

/* Authors: David P. Leins */

#include <mujoco_ros_sensors/serialization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace mujoco_ros::sensors {

void serialize(geometry_msgs::QuaternionStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff)
{
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id;
	msg.quaternion.w    = static_cast<double>(data->sensordata[adr] / cutoff);
	msg.quaternion.x    = static_cast<double>(data->sensordata[adr + 1] / cutoff);
	msg.quaternion.y    = static_cast<double>(data->sensordata[adr + 2] / cutoff);
	msg.quaternion.z    = static_cast<double>(data->sensordata[adr + 3] / cutoff);
}

void serialize(geometry_msgs::Vector3Stamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff)
{
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id;
	msg.vector.x        = static_cast<double>(data->sensordata[adr] / cutoff);
	msg.vector.y        = static_cast<double>(data->sensordata[adr + 1] / cutoff);
	msg.vector.z        = static_cast<double>(data->sensordata[adr + 2] / cutoff);
}

void serialize(geometry_msgs::PointStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff)
{
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id;
	msg.point.x         = static_cast<double>(data->sensordata[adr] / cutoff);
	msg.point.y         = static_cast<double>(data->sensordata[adr + 1] / cutoff);
	msg.point.z         = static_cast<double>(data->sensordata[adr + 2] / cutoff);
}

void serialize(mujoco_ros_msgs::ScalarStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff)
{
	msg.header.stamp    = ros::Time::now();
	msg.header.frame_id = frame_id;
	msg.value           = static_cast<double>(data->sensordata[adr] / cutoff);
}

///////////// Noise addition ///////////////

////// Quaternion //////

void addNoise(geometry_msgs::Quaternion &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}

	// For correct application, generate noise in roll, pitch, yaw, then convert to quaternion
	tf2::Quaternion q, q_noise;
	tf2::fromMsg(msg, q);
	q.normalize();

	double r = 0., p = 0., y = 0.;
	int noise_idx = 0;
	if (noise_set & 0x01) { // Noise in roll
		r = dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x02) { // Noise in pitch
		p = dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x04) { // Noise in yaw
		y = dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
	}
	q_noise.setRPY(r, p, y);

	// Rotate original quaternion by noise quaternion
	msg = tf2::toMsg((q_noise * q).normalize());
}

void addNoise(geometry_msgs::QuaternionStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}
	addNoise(msg.quaternion, mu_noise, sigma_noise, noise_set, dist, gen);
}

////// Vector3 //////

void addNoise(geometry_msgs::Vector3 &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}

	int noise_idx = 0;
	if (noise_set & 0x01) { // Noise in x
		msg.x += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x02) { // Noise in y
		msg.y += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x04) { // Noise in z
		msg.z += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
	}
}

void addNoise(geometry_msgs::Vector3Stamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}
	addNoise(msg.vector, mu_noise, sigma_noise, noise_set, dist, gen);
}

////// Point //////

void addNoise(geometry_msgs::Point &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}

	int noise_idx = 0;
	if (noise_set & 0x01) { // Noise in x
		msg.x += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x02) { // Noise in y
		msg.y += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
		noise_idx++;
	}
	if (noise_set & 0x04) { // Noise in z
		msg.z += dist(gen) * sigma_noise[noise_idx] + mu_noise[noise_idx];
	}
}

void addNoise(geometry_msgs::PointStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}
	addNoise(msg.point, mu_noise, sigma_noise, noise_set, dist, gen);
}

////// Scalar //////

void addNoise(double &value, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}

	if (noise_set & 0x01) { // Noise in x
		// shift and scale standard normal to desired distribution
		value += dist(gen) * sigma_noise[0] + mu_noise[0];
	}
}

void addNoise(mujoco_ros_msgs::ScalarStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen)
{
	if (noise_set == 0) { // no noise, leave unchanged
		return;
	}
	addNoise(msg.value, mu_noise, sigma_noise, noise_set, dist, gen);
}

} // namespace mujoco_ros::sensors
