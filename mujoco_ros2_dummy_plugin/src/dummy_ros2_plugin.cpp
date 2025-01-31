#include "mujoco_ros2_dummy_plugin/dummy_ros2_plugin.hpp"

namespace mujoco_ros {

    DummyRos2Plugin::~DummyRos2Plugin(){

        RCLCPP_INFO_STREAM(get_my_logger(), "Deleting DummyRos2Plugin");
        executor_->remove_node(my_node_);
        executor_->cancel();
        thread_executor_spin_.join();
    }
    void DummyRos2Plugin::ControlCallback(const mjModel* model, mjData* data){
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        publisher_->publish(message);
    }
	void DummyRos2Plugin::PassiveCallback(const mjModel* model, mjData* data){

    }
	void DummyRos2Plugin::RenderCallback(const mjModel* model, mjData* data, mjvScene* scene){

    }
	void DummyRos2Plugin::LastStageCallback(const mjModel* model, mjData* data){

    }
	void DummyRos2Plugin::OnGeomChanged(const mjModel* model, mjData* data, const int geom_id){

    }
    bool DummyRos2Plugin::Load(const mjModel *m, mjData *d){
        m_ = m;
        d_ = d;
        count_ = 0;
        my_node_ = std::make_shared<rclcpp::Node>("dummy_node"); 
        RCLCPP_INFO_STREAM(get_my_logger(), "Fully qualified name: " << my_node_->get_fully_qualified_name());
        publisher_ = my_node_->create_publisher<std_msgs::msg::String>("topic", 10);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(my_node_);
        auto spin = [this]()
            {
            this->executor_->spin();
            };
        thread_executor_spin_ = std::thread(spin);
        return true;
    }
    void DummyRos2Plugin::Reset(){

    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros::DummyRos2Plugin, mujoco_ros::MujocoPlugin)
