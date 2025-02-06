#include "mujoco_ros2_dummy_plugin/dummy_ros2_plugin.hpp"

namespace mujoco_ros {

    DummyRos2Plugin::~DummyRos2Plugin(){

        RCLCPP_INFO_STREAM(get_my_logger(), "Deleting DummyRos2Plugin");
        // Since we've added the sub-nodes to the executor manually, we need to remove them manually as well
        env_ptr_->RemoveNodeFromExecutor(parallel_node_->get_node_base_interface());
        env_ptr_->RemoveNodeFromExecutor(child_node_->get_node_base_interface());
    }

    void DummyRos2Plugin::ControlCallback(const mjModel* model, mjData* data){
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        publisher_->publish(message);

        auto message_child = std_msgs::msg::String();
        message.data = "Hello, world from child! " + std::to_string(count_);
        child_publisher_->publish(message_child);
    }
 
    // These don't need to be implemented if they are not used
	// void DummyRos2Plugin::PassiveCallback(const mjModel* model, mjData* data){

    // }
	// void DummyRos2Plugin::RenderCallback(const mjModel* model, mjData* data, mjvScene* scene){

    // }
	// void DummyRos2Plugin::LastStageCallback(const mjModel* model, mjData* data){

    // }
	// void DummyRos2Plugin::OnGeomChanged(const mjModel* model, mjData* data, const int geom_id){

    // }

    mujoco_ros::CallbackReturn DummyRos2Plugin::on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
        RCLCPP_INFO_STREAM(get_my_logger(), "Configuring DummyRos2Plugin");

        declare_parameter_if_not_declared(
            this->get_node()->get_node_parameters_interface(),
            "parallel_name",
            rclcpp::ParameterValue("parallel_node_name")
        );
        declare_parameter_if_not_declared(
            this->get_node()->get_node_parameters_interface(),
            "child_name",
            rclcpp::ParameterValue("thisIsReplacedByTheConfigFile")
        );

        return mujoco_ros::CallbackReturn::SUCCESS;
    }

    bool DummyRos2Plugin::Load(const mjModel *m, mjData *d){
        m_ = m;
        d_ = d;
        count_ = 0;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_handle = get_node();

        RCLCPP_INFO_STREAM(get_my_logger(), "Namespace: " << node_handle->get_namespace() << "; Fully qualified name: " << node_handle->get_node_base_interface()->get_fully_qualified_name());

        // This plugin itself now implements a LifecycleNodeInterface and is automatically added to the parent executor
        // we can directly create publishers and subscribers
        publisher_ = get_node()->create_publisher<std_msgs::msg::String>("topic", 10);

        // make sure to first declare parameters in the `on_configure` function,
        // then get them in the `Load` function
        std::string parallel_name = node_handle->get_parameter("parallel_name").as_string();

        // In this case you would probably want to create a new class instance derived from Node or LifecycleNode
        // with a pluginlib loader 
        // we need to pass the remap argument for the namespace to not be broken
        auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", parallel_name + ":__node:=" + parallel_name});
        parallel_node_ = std::make_shared<rclcpp::Node>(parallel_name, options);

        // this time adding the node to the executor needs to be done manually
        env_ptr_->AddNodeToExecutor(parallel_node_->get_node_base_interface());

        // Then, in case it's a LifecycleNode and provides a configuration hook, you would run the 
        // `on_configure` function here to declare parameters and other interfaces
        // child_node_->configure();

        // After this step the node should be ready to run. In this case were just creating a publisher
        parallel_publisher_ = parallel_node_->create_publisher<std_msgs::msg::String>("parallel_topic", 10);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("ParallelNode"), "Namespace: " << parallel_node_->get_namespace() << "; Fully qualified name: " << parallel_node_->get_node_base_interface()->get_fully_qualified_name());

        /// Now a child node, i.e., a node that is nested within the parent node's namespace

        const std::string child_name = node_handle->get_parameter("child_name").as_string();

        const auto child_options = rclcpp::NodeOptions().arguments({"--ros-args", "--remap", child_name + ":__node:=" + child_name});

        // This is in practice more complex, because we have to check first if the node is in a namespace itself and then we add 
        // the node name plus the child name. But since we know how things are set up in this example, we just do it quick and dirty
        const std::string ns = std::string(node_handle->get_name());

        child_node_ = std::make_shared<rclcpp::Node>(child_name, ns, child_options);
        child_publisher_ = child_node_->create_publisher<std_msgs::msg::String>("child_topic", 10);

        RCLCPP_INFO_STREAM(rclcpp::get_logger("ChildNode"), "Namespace: " << child_node_->get_namespace() << "; Fully qualified name: " << child_node_->get_node_base_interface()->get_fully_qualified_name());

        return true;
    }

    void DummyRos2Plugin::Reset(){

    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros::DummyRos2Plugin, mujoco_ros::MujocoPlugin)
