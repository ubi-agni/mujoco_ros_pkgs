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

/* Authors: Julian Leichert*/
#include <mujoco_ros_hog/mujoco_ros_hog_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <tf/transform_datatypes.h>


namespace mujoco_ros_hog {


bool MujocoHogPlugin::load(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{

	// Check that ROS has been initialized
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM_NAMED("mujoco_ros_hog",
		                       "A ROS node for Mujoco has not been initialized, unable to load plugin.");
		return false;
	}

	ROS_ASSERT(rosparam_config_.getType() == XmlRpc::XmlRpcValue::TypeStruct);
	if (!rosparam_config_.hasMember("hogBodies")) {
		ROS_ERROR_NAMED("mujoco_ros_hog", "MujocoRosHogPlugin expects a 'hogBodies' rosparam specifying at least "
		                                      "one body");
		return false;
	}
	ROS_ASSERT(rosparam_config_["hogBodies"].getType() == XmlRpc::XmlRpcValue::TypeStruct);

	XmlRpc::XmlRpcValue::iterator itr;

	for (itr = rosparam_config_["hogBodies"].begin(); itr != rosparam_config_["hogBodies"].end(); ++itr) {

		if(itr->second.hasMember("desiredPose")){
			ROS_ASSERT(itr->second["desiredPose"].getType() == XmlRpc::XmlRpcValue::TypeArray);
			std::vector<double> pose;
			for (int32_t i = 0; i < itr->second["desiredPose"].size(); ++i) 
			{
				ROS_ASSERT(itr->second["desiredPose"][i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
				pose.push_back(itr->second["desiredPose"][i]);
			}
			ROS_INFO_STREAM_NAMED("mujoco_ros_hog", "Found hog object, with a defined position, named: " << itr->first);
			registerHog(m,itr->first,pose);
		}else{
			ROS_INFO_STREAM_NAMED("mujoco_ros_hog", "Found hog object with name: "<<itr->first);
			ROS_WARN_STREAM_NAMED("mujoco_ros_hog","MujocoRosHogPlugin didn't read a desired Position for the body: " << itr->first);
			registerHog(m,itr->first);
		}
    }

	tf_buffer_.reset(new tf2_ros::Buffer());
	ROS_INFO_NAMED("mujoco_ros_hog", "Hog initialized");
	return true; 
	
}

void MujocoHogPlugin::controlCallback(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	updateHog(m,d);
}

void MujocoHogPlugin::updateHog(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
    for (std::map<std::string,std::vector<double>>::iterator it = hog_bodies_.begin() ; it != hog_bodies_.end(); ++it) {

		std::string body_name = it->first;
		geometry_msgs::TransformStamped hog_desired_tform;
		tf::Vector3 p;
		tf::Quaternion q;
		if(!it->second.empty()&& it->second.size()==7){
            p = tf::Vector3(it->second[0],it->second[1],it->second[2]);
            q = tf::Quaternion(it->second[4],it->second[5],it->second[6],it->second[3]);

		}else{
			try{
				hog_desired_tform = tf_buffer_->lookupTransform("hog_desired", body_name, ros::Time(0));
			} catch (tf2::TransformException ex){
				ROS_ERROR_ONCE("%s",ex.what());
				return;
			}
            const geometry_msgs::Vector3 pm = hog_desired_tform.transform.translation;
            const geometry_msgs::Quaternion qm = hog_desired_tform.transform.rotation;  
            p = tf::Vector3(pm.x,pm.y,pm.z);
            q = tf::Quaternion(qm.x,qm.y,qm.z,qm.w);
		}
		// Convert TF transform to Gazebo Pose
		unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, body_name.c_str());
		unsigned int h;
		double kl = 2500;
		double ka = 250;
		double cl = 2.0 * sqrt(kl * m->body_subtreemass[fidx]);
		double ca = 2.0 * sqrt(ka * m->body_inertia[3*fidx]);
		
		
		const tf::Vector3 pc(d->xpos[3*fidx    ],
							d->xpos[3*fidx + 1],
							d->xpos[3*fidx + 2]);
		tf::Quaternion qc(d->xquat[4*fidx + 1],
								d->xquat[4*fidx + 2],
								d->xquat[4*fidx + 3],
								d->xquat[4*fidx    ]);
		
		qc = qc.normalized();
		mjtNum vx[6];
		const tf::Vector3 pe = p - pc;
		tf::Quaternion qe = q * qc.inverse();
		mj_objectVelocity(m.get(),d.get(),mjOBJ_XBODY, fidx, vx, 0);
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6*fidx + i] = kl * pe.m_floats[i]- cl *vx[i+3];
		}
		d->xfrc_applied[6*fidx + 2] += 9.81 * m->body_subtreemass[fidx];
		tf::Vector3 ql(0,0,0);
		if (std::fabs(qe.w() < 1.0)) {
			double acosw = std::acos(qe.w());
			double qsin = std::sin(acosw);
			if (std::fabs(qsin) <= 1e-3) {
			double c = acosw/qsin;
			ql.setX(c*qe.x());
			ql.setY(c*qe.y());
			ql.setZ(c*qe.z());
			}
		}
		
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6*fidx + i + 3] = ka * ql.m_floats[i] - ca * vx[i];
		}
	}

}


bool MujocoHogPlugin::registerHog(MujocoSim::mjModelPtr m, std::string bodyname,std::vector<double> desiredPose)
{
    int id = mj_name2id(m.get(), mjOBJ_XBODY, bodyname.c_str());
    // check if the body exists
    if (id == -1) {
        ROS_ERROR_STREAM_NAMED("mujoco_ros_hog","Could not find hog body with the name:"<<bodyname);
        return false;
    } else {
        ROS_INFO_STREAM_NAMED("mujoco_ros_hog","Registered hog body with the name:"<<bodyname);
        hog_bodies_.insert(std::pair<std::string,std::vector<double>>(bodyname,desiredPose));
    }
    return true;
}

void MujocoHogPlugin::reset() {};
}//namespace mujoco_ros_hog

PLUGINLIB_EXPORT_CLASS(mujoco_ros_hog::MujocoHogPlugin, MujocoSim::MujocoPlugin)
