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

	if(d->time != last_time){
		updateHog(m,d);
	}
	last_time = d->time;
}

void MujocoHogPlugin::updateHog(MujocoSim::mjModelPtr m, MujocoSim::mjDataPtr d)
{
	int env_id = 0;
	MujocoSim::MujocoEnvPtr env = MujocoSim::environments::getEnv(d.get());
	
	double g;
	if (env != nullptr) {
		g = env->model_->opt.gravity[2];
	}else{
		ROS_ERROR_STREAM_NAMED("mujoco_ros_hog","Couldn't get Environment with ID: " << env_id);
		return;
	}
	
    for (std::map<std::string,std::vector<double>>::iterator it = hog_bodies_.begin() ; it != hog_bodies_.end(); ++it) {

		std::string body_name = it->first;
		geometry_msgs::TransformStamped hog_desired_tform;

		geometry_msgs::Vector3 pm;
		geometry_msgs::Quaternion qm;  
		if(!it->second.empty()&& it->second.size()==7){
            pm.x = it->second[0];
			pm.y = it->second[1];
			pm.z = it->second[2];
            qm.w = it->second[3];
			qm.x = it->second[4];
			qm.y = it->second[5];
			qm.z = it->second[6];

		}else{
			try{
				hog_desired_tform = tf_buffer_->lookupTransform("hog_desired", body_name, ros::Time(0));
			} catch (tf2::TransformException ex){
				ROS_ERROR_ONCE("%s",ex.what());
				return;
			}
            pm = hog_desired_tform.transform.translation;
            qm = hog_desired_tform.transform.rotation;  
		}
		unsigned int fidx = mj_name2id(m.get(), mjOBJ_XBODY, body_name.c_str());
		unsigned int h;
		double kl = 2500;
		double ka = 250;
		double cl = 2.0 * sqrt(kl * m->body_subtreemass[fidx]);
		double ca = 2.0 * sqrt(ka * m->body_inertia[3*fidx]);
		
		
		mjtNum p[3] = {pm.x,pm.y,pm.z};
		mjtNum q[4] = {qm.w,qm.x,qm.y,qm.z};
		mjtNum pc[3] = {d->xpos[3*fidx    ],
							d->xpos[3*fidx + 1],
							d->xpos[3*fidx + 2]};
		
		
		mjtNum qc[4] = {d->xquat[4*fidx],
								d->xquat[4*fidx + 1],
								d->xquat[4*fidx + 2],
								d->xquat[4*fidx +3]}; 
		mju_normalize4(qc);
		mjtNum vx[6];
		mjtNum pe[3];
		mjtNum qe[4];
		mjtNum qi[4];
		mju_sub3(pe,p,pc); 
		mju_negQuat(qi,qc);
		mju_mulQuat(qe,q,qi);
		mj_objectVelocity(m.get(),d.get(),mjOBJ_XBODY, fidx, vx, 0);
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6*fidx + i] = kl * pe[i]- cl *vx[i+3];
		}
		d->xfrc_applied[6*fidx + 2] += -1* g * m->body_subtreemass[fidx];
		mjtNum ql[3] = {0,0,0};
		if (qe[0] < 1.0) {
			double acosw = std::acos(qe[1]);
			double qsin = std::sin(acosw);
			if (std::fabs(qsin) <= 1e-3) {
				double c = acosw/qsin;
				ql[1] = c*qe[1];
				ql[2] = c*qe[2];
				ql[3] = c*qe[3];
			}
		}
		for (int i = 0; i < 3; ++i) {
			d->xfrc_applied[6*fidx + i + 3] = ka * ql[i] - ca * vx[i];
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
