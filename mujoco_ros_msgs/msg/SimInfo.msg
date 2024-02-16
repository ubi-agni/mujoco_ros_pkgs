string model_path 
bool model_valid
uint16 load_count # counter of (re)loads
mujoco_ros_msgs/StateUint loading_state
bool paused
uint16 pending_sim_steps
float32 rt_measured # measured real-time factor
float32 rt_setting # desired real-time factor