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

#pragma once

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <mujoco_ros_msgs/ScalarStamped.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjtnum.h>

#include <random>

namespace mujoco_ros::sensors {

void serialize(geometry_msgs::QuaternionStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff);
void serialize(geometry_msgs::Vector3Stamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff);
void serialize(geometry_msgs::PointStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff);
void serialize(mujoco_ros_msgs::ScalarStamped &msg, const mjData *data, const std::string &frame_id, const int &adr,
               const mjtNum &cutoff);

void addNoise(geometry_msgs::Quaternion &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen);
void addNoise(geometry_msgs::QuaternionStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen);

void addNoise(geometry_msgs::Vector3 &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen);
void addNoise(geometry_msgs::Vector3Stamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen);

void addNoise(geometry_msgs::Point &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen);
void addNoise(geometry_msgs::PointStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen);

void addNoise(double &value, const mjtNum *mu_noise, const mjtNum *sigma_noise, const uint8_t &noise_set,
              std::normal_distribution<double> &dist, std::mt19937 &gen);
void addNoise(mujoco_ros_msgs::ScalarStamped &msg, const mjtNum *mu_noise, const mjtNum *sigma_noise,
              const uint8_t &noise_set, std::normal_distribution<double> &dist, std::mt19937 &gen);

} // namespace mujoco_ros::sensors
