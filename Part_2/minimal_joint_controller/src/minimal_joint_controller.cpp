#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ignition/transport.hh>
#include <std_msgs/msg/float64.hpp>
#include <math.h>
#include <string>

const double Kp = 10.0; //controller gains
const double Kv = 3;
const double dt = 0.01;

double g_pos_cmd = 0.0; //position command input-- global var

// Saturation function
double sat(double val, double sat_val) {
    if (val > sat_val) return sat_val;
    if (val < -sat_val) return -sat_val;
    return val;
}

double min_periodicity(double theta_val) {
    double periodic_val = theta_val;
    while (periodic_val > M_PI) {
        periodic_val -= 2 * M_PI;
    }
    while (periodic_val < -M_PI) {
        periodic_val += 2 * M_PI;
    }
    return periodic_val;
}

// Controller node class
class MinimalJointController : public rclcpp::Node {
public:
    MinimalJointController() : Node("minimal_joint_controller") {
        // Declare and initialize publishers
        trq_publisher_ = this->create_publisher<std_msgs::msg::Float64>("jnt_trq", 1);
        vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>("jnt_vel", 1);
        pos_publisher_ = this->create_publisher<std_msgs::msg::Float64>("jnt_pos", 1);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

        // Declare and initialize subscriber
        pos_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "pos_cmd", 1, std::bind(&MinimalJointController::posCmdCB, this, std::placeholders::_1));

        joint_state_msg_.header.stamp = this->now();
        joint_state_msg_.name.push_back("joint1");
        joint_state_msg_.position.push_back(0.0);
        joint_state_msg_.velocity.push_back(0.0);

        // Timer for the main loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt * 1000)),
            std::bind(&MinimalJointController::controllerLoop, this));

        // Ensure services are available
       // while (!test_services()) {
           // RCLCPP_WARN(this->get_logger(), "Waiting for services...");
           // rclcpp::sleep_for(std::chrono::seconds(1));
       // }
    }

private:
    // Callback to receive position command
    void posCmdCB(const std_msgs::msg::Float64::SharedPtr pos_cmd_msg) {
        RCLCPP_INFO(this->get_logger(), "Received position command: %f", pos_cmd_msg->data);
        g_pos_cmd = pos_cmd_msg->data;
    }

    void applyJointEffortCB(const ignition::msgs::Boolean &response) {
	if (!response.data()) {
		RCLCPP_WARN(this->get_logger(), "Service call to apply_joint_effort failed!");
	}
    }

    void getJointPropertiesCB(const ignition::msgs::Double_V &response){
	if(response.data_size() >= 2){
		auto q1 = response.data(0);
		auto q1dot = response.data(1);

		std_msgs::msg::Float64 q1_msg;
		q1_msg.data = q1;
		pos_publisher_->publish(q1_msg);

		std_msgs::msg::Float64 q1dot_msg;
		q1dot_msg.data = q1dot;
		vel_publisher_->publish(q1dot_msg);

		joint_state_msg_.header.stamp = this->now();
		joint_state_msg_.position[0] = q1;
		joint_state_msg_.velocity[0] = q1dot;
		joint_state_publisher_->publish(joint_state_msg_);

		double q1_err = min_periodicity(g_pos_cmd - q1);
		double trq_cmd = Kp * q1_err - Kv * q1dot;

		std_msgs::msg::Float64 trq_msg;
		trq_msg.data = trq_cmd;
		trq_publisher_->publish(trq_msg);

		ignition::msgs::Double trq_cmd_msg;
		trq_cmd_msg.set_data(trq_cmd);
		ignNode.Request("/apply_joint_effort", trq_cmd_msg);
	} else {
		RCLCPP_WARN(this->get_logger(), "Failed to call get_joint_properties service");
	}
    }
    // Controller loop
    void controllerLoop() {
        
	ignition::msgs::StringMsg req;
	req.set_data("joint1");
	ignNode.Request("/get_joint_properties", req);
	//ignNode.Request("/world/default/get_joint_properties", req, std::bind(&MinimalJointController::getJointPropertiesCB, this, std::placeholders::_1));
    }

    // Members
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trq_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pos_cmd_subscriber_;

    sensor_msgs::msg::JointState joint_state_msg_;
    ignition::transport::Node ignNode;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalJointController>());
    rclcpp::shutdown();
    return 0;
}

