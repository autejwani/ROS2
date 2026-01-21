// minimal_controller node: 
// wsn example node that both subscribes and publishes--counterpart to minimal_simulator 
// subscribes to "velocity" and publishes "force_cmd" 
// subscribes to "vel_cmd" 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

//global variables for callback functions to populate for use in main program 
std_msgs::msg::Float64 g_velocity;
std_msgs::msg::Float64 g_vel_cmd;
std_msgs::msg::Float64 g_force; // this one does not need to be global... 

void myCallbackVelocity(const std_msgs::msg::Float64::SharedPtr message_holder, rclcpp::Logger logger) {
    // check for data on topic "velocity" 
    RCLCPP_INFO(logger, "received velocity value is: %f", message_holder->data);
    g_velocity.data = message_holder->data; // post the received data in a global var for access by 
    //main prog. 
}

void myCallbackVelCmd(const std_msgs::msg::Float64::SharedPtr message_holder, rclcpp::Logger logger) {
    // check for data on topic "vel_cmd" 
    RCLCPP_INFO(logger, "received velocity command value is: %f", message_holder->data);
    g_vel_cmd.data = message_holder->data; // post the received data in a global var for access by 
    //main prog. 
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS 2
    auto node = rclcpp::Node::make_shared("minimal_controller"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_controller" 
    
    auto logger = node->get_logger();
    
    //create 2 subscribers: one for state sensing (velocity) and one for velocity commands 
    auto my_subscriber_object1 = node->create_subscription<std_msgs::msg::Float64>(
        "velocity", 10, 
        [&logger](const std_msgs::msg::Float64::SharedPtr msg) {
            myCallbackVelocity(msg, logger);
        });
    auto my_subscriber_object2 = node->create_subscription<std_msgs::msg::Float64>(
        "vel_cmd", 10,
        [&logger](const std_msgs::msg::Float64::SharedPtr msg) {
            myCallbackVelCmd(msg, logger);
        });
    //publish a force command computed by this controller; 
    auto my_publisher_object = node->create_publisher<std_msgs::msg::Float64>("force_cmd", 10);
    double Kv = 1.0; // velocity feedback gain 
    double dt_controller = 0.1; //specify 10Hz controller sample rate (pretty slow, but 
    //illustrative) 
    double sample_rate = 1.0 / dt_controller; // compute the corresponding update frequency 
    rclcpp::Rate naptime(sample_rate); // use to regulate loop rate 
    g_velocity.data = 0.0; //initialize velocity to zero 
    g_force.data = 0.0; // initialize force to 0; will get updated by callback 
    g_vel_cmd.data = 0.0; // init velocity command to zero 
    double vel_err = 0.0; // velocity error 
    // enter the main loop: get velocity state and velocity commands 
    // compute command force to get system velocity to match velocity command 
    // publish this force for use by the complementary simulator 
    while (rclcpp::ok()) {
        rclcpp::spin_some(node); //allow data update from callback; must come FIRST to get latest values
        vel_err = g_vel_cmd.data - g_velocity.data; // compute error btwn desired and actual 
        //velocities 
        g_force.data = Kv*vel_err; //proportional-only velocity-error feedback defines commanded 
        //force 
        my_publisher_object->publish(g_force); // publish the control effort computed by this 
        //controller 
        RCLCPP_INFO(logger, "force command = %f, vel_cmd = %f, velocity = %f", g_force.data, g_vel_cmd.data, g_velocity.data);
        naptime.sleep(); // wait for remainder of specified period; 
    }
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0; // should never get here, unless ros2 core dies 
} 
