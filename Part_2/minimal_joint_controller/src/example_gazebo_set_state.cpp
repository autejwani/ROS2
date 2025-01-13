#include <rclcpp/rclcpp.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/twist.pb.h>

class SetMotionNode : public rclcpp::Node
{
public:
    SetMotionNode() : Node("set_motion_node")
    {
        ign_node_ = std::make_shared<ignition::transport::Node>();
        
        // Initialize the publisher once in the constructor
        pub_ = ign_node_->Advertise<ignition::msgs::Twist>("/model/rect_prism/cmd_vel");

        // Set up the timer to repeatedly send the velocity command
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&SetMotionNode::sendVelocityCommand, this));
    }

private:
    void sendVelocityCommand()
    {
        ignition::msgs::Twist velocity_cmd;
        velocity_cmd.mutable_linear()->set_x(1.0); // m/s
        velocity_cmd.mutable_angular()->set_z(1.0); // rad/s

        // Publish the velocity command using the Publisher object
        pub_.Publish(velocity_cmd);
        
        RCLCPP_INFO(this->get_logger(), "Published velocity command: linear x=0.01, angular z=1.0");
    }

    std::shared_ptr<ignition::transport::Node> ign_node_;
    ignition::transport::Node::Publisher pub_; // Store the publisher object
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetMotionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

