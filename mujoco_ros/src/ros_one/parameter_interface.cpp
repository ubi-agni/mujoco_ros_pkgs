#include <mujoco_ros/ros_version.hpp>
#include <mujoco_ros/render_backend.hpp>
#include <mujoco_ros/logging.hpp>

#include <ros/ros.h>

#include <mujoco_ros/mujoco_env.hpp>
#include <mujoco_ros/ros_one/plugin_utils.hpp>

// #include <mujoco_ros/offscreen_camera.hpp>

#include <mujoco/mujoco.h>
#include <mujoco_ros/array_safety.h>

#include <rosgraph_msgs/Clock.h>

namespace mju = ::mujoco::sample_util;

namespace mujoco_ros {

void MujocoEnv::FetchRosConfiguration()
{
	ROS_DEBUG("Fetching configuration");

	bool eval_mode = false;
	nh_->param("eval_mode", eval_mode, false);
	settings_.eval_mode = eval_mode;
	if (!ros::param::get("/use_sim_time", settings_.use_sim_time)) {
		ROS_FATAL("/use_sim_time ROS param is unset. This node requires you to explicitly set it to true "
		          "or false. Also Make sure it is set before starting any node, "
		          "otherwise nodes might behave unexpectedly.");
		throw std::runtime_error("/use_sim_time ROS param is unset.");
	}
	bool no_render;
	nh_->param("no_render", no_render, false);
	if (nh_->hasParam("no_x")) {
		ROS_WARN("The 'no_x' parameter is deprecated. Use 'no_render' instead.");
		nh_->param("no_x", no_render, no_render);
	}

	if (no_render) {
		ROS_INFO("no_render is set. Disabling rendering and setting headless to true");
		nh_->setParam("headless", true);
		nh_->setParam("render_offscreen", false);
	}

	nh_->param("render_offscreen", settings_.render_offscreen, true);
	nh_->param("headless", settings_.headless, true);
	nh_->param("num_steps", num_steps_until_exit_, -1);
	bool run; // settings_.run is atomic and can't directly be set!
	nh_->param("unpause", run, true);
	settings_.run = run;

	/*
	 * Model (file) passing: the model can be provided as file to parse or directly as string stored in the rosparam
	 * server. If both string and file are provided, the string takes precedence.
	 */
	std::string filename = "";
	nh_->getParam("modelfile", filename);
	std::string xml_content_path;
	std::string xml_content;

	bool wait_for_xml;
	nh_->param("wait_for_xml", wait_for_xml, false);

	ROS_INFO_COND(wait_for_xml, "Waiting for xml content to be available on rosparam server");

	// TODO: add timeout of 60 seconds
	while (wait_for_xml) {
		if (nh_->searchParam("mujoco_xml", xml_content_path) || ros::param::search("mujoco_xml", xml_content_path)) {
			ROS_DEBUG_STREAM("Found mujoco_xml_content param under " << xml_content_path);

			nh_->getParam(xml_content_path, xml_content);
			if (!xml_content.empty()) {
				ROS_INFO("Got xml content from ros param server");
				ROS_WARN_COND(!filename.empty(),
				              "Both xml content and modelfile supplied as parameters! Using xml content.");
				filename = "rosparam_content";
			}
			wait_for_xml = false;
		}
	}

	ROS_INFO_STREAM_COND(!filename.empty(), "Using modelfile " << filename);
	ROS_WARN_COND(filename.empty(), "No modelfile was provided, launching empty simulation!");

	if (!filename.empty()) {
		mju::strcpy_arr(queued_filename_, filename.c_str());
		settings_.load_request = 2;
	}
	// TODO: after python bindings merge
	// nh_->param<int>("num_mj_threads", settings_.num_threads, 4);
}

void MujocoEnv::InitTFBroadcasting()
{
	static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>();
	tf_buffer_          = std::make_unique<tf2_ros::Buffer>();
	tf_buffer_->setUsingDedicatedThread(true);
	tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void MujocoEnv::GetCameraConfiguration(const std::string &cam_name, rendering::StreamType &stream_type,
                                       float &pub_frequency, bool &use_segid, int &width, int &height,
                                       std::string &base_topic, std::string &rgb_topic, std::string &depth_topic,
                                       std::string &segment_topic)
{
	bool config_exists;
	std::string cam_config_path;
	config_exists = nh_->searchParam("cam_config", cam_config_path) || ros::param::search("cam_config", cam_config_path);
	MJR_DEBUG_STREAM_COND(config_exists, "Found camera config under path: " << cam_config_path);

	stream_type = rendering::StreamType(
	    nh_->param<int>(cam_config_path + "/" + cam_name + "/stream_type", rendering::kDEFAULT_CAM_STREAM_TYPE));
	pub_frequency = nh_->param<float>(cam_config_path + "/" + cam_name + "/frequency", rendering::kDEFAULT_CAM_PUB_FREQ);
	use_segid     = nh_->param<bool>(cam_config_path + "/" + cam_name + "/use_segid", rendering::kDEFAULT_CAM_USE_SEGID);
	width         = nh_->param<int>(cam_config_path + "/" + cam_name + "/width", rendering::kDEFAULT_CAM_WIDTH);
	height        = nh_->param<int>(cam_config_path + "/" + cam_name + "/height", rendering::kDEFAULT_CAM_HEIGHT);
	base_topic    = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/topic", "cameras/" + cam_name);
	rgb_topic     = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_rgb",
                                       std::string(rendering::kDEFAULT_CAM_RGB_TOPIC));
	depth_topic   = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_depth",
                                         std::string(rendering::kDEFAULT_CAM_DEPTH_TOPIC));
	segment_topic = nh_->param<std::string>(cam_config_path + "/" + cam_name + "/name_segment",
	                                        std::string(rendering::kDEFAULT_CAM_SEGMENT_TOPIC));
}

void MujocoEnv::GetInitialJointPositions(std::map<std::string, std::vector<double>> &joint_pos_map)
{
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_positions/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_positions/joint_map") && joint_map.empty()) {
		MJR_WARN("Initial joint positions not recognized by rosparam server. Check your config, "
		         "especially values for single axis joints should explicitly provided as string!");
		return;
	}

	for (auto const &[name, str_values] : joint_map) {
		MJR_DEBUG_STREAM("fetched jointpos values of joint " << name << ": " << str_values);

		std::vector<double> axis_vals;
		axis_vals.reserve(7);

		std::stringstream stream_values(str_values);
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			axis_vals.push_back(std::stod(value));
		}
		axis_vals.shrink_to_fit();
		joint_pos_map[name] = axis_vals;
	}
}

void MujocoEnv::GetInitialJointVelocities(std::map<std::string, std::vector<double>> &joint_vel_map)
{
	std::map<std::string, std::string> joint_map;
	nh_->getParam("initial_joint_velocities/joint_map", joint_map);

	// This check only assures that there aren't single axis joint values that are non-strings.
	// One ill-defined value among correct parameters can't be detected.
	if (nh_->hasParam("initial_joint_velocities/joint_map") && joint_map.empty()) {
		MJR_WARN("Initial joint velocities not recognized by rosparam server. Check your config, "
		         "especially values for single axis joints should explicitly provided as string!");
		return;
	}

	for (auto const &[name, str_values] : joint_map) {
		MJR_DEBUG_STREAM("fetched jointvel values of joint " << name << ": " << str_values);

		std::vector<double> axis_vals;
		axis_vals.reserve(7);

		std::stringstream stream_values(str_values);
		std::string value;
		while (std::getline(stream_values, value, ' ')) {
			axis_vals.push_back(std::stod(value));
		}
		axis_vals.shrink_to_fit();
		joint_vel_map[name] = axis_vals;
	}
}

// TODO:
// setupServices
// callbacks

} // namespace mujoco_ros
