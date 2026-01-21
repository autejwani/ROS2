// minimal_simulator node: 
// wsn example node that both subscribes and publishes 
// does trivial system simulation, F=ma, to update velocity given F specified on topic "force_cmd" 
// publishes velocity on topic "velocity" 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <thread>

std_msgs::msg::Float64 g_velocity;
std_msgs::msg::Float64 g_force;

void myCallback(const std_msgs::msg::Float64::SharedPtr message_holder, rclcpp::Logger logger) {
    // checks for messages on topic "force_cmd" 
    RCLCPP_INFO(logger, "received force value is: %f", message_holder->data);
    g_force.data = message_holder->data; // post the received data in a global var for access by 
    // main prog. 
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS 2
    auto node = rclcpp::Node::make_shared("minimal_simulator"); //name this node 
    // when this compiled code is run, ROS will recognize it as a node called "minimal_simulator" 
    
    auto logger = node->get_logger();
    
    //create a Subscriber object and have it subscribe to the topic "force_cmd" 
    auto my_subscriber_object = node->create_subscription<std_msgs::msg::Float64>(
        "force_cmd", 10,
        [&logger](const std_msgs::msg::Float64::SharedPtr msg) {
            myCallback(msg, logger);
        });
    //simulate accelerations and publish the resulting velocity; 
    auto my_publisher_object = node->create_publisher<std_msgs::msg::Float64>("velocity", 10);
    double mass = 1.0;
    double dt = 0.01; //10ms integration time step 
    double sample_rate = 1.0 / dt; // compute the corresponding update frequency 
    rclcpp::Rate naptime(sample_rate);
    g_velocity.data = 0.0; //initialize velocity to zero 
    g_force.data = 0.0; // initialize force to 0; will get updated by callback 
    
    // Give time for publisher/subscriber to be discovered by other nodes
    // Do a few spin cycles to allow discovery
    for (int i = 0; i < 5; i++) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Publish an initial message to advertise the topic
    my_publisher_object->publish(g_velocity);
    RCLCPP_INFO(logger, "Publishing velocity topic (initial value: %f)", g_velocity.data);
    
    while (rclcpp::ok()) {
        g_velocity.data = g_velocity.data + (g_force.data / mass) * dt; // Euler integration of 
        //acceleration 
        my_publisher_object->publish(g_velocity); // publish the system state (trivial--1-D) 
        RCLCPP_INFO(logger, "velocity = %f", g_velocity.data);
        rclcpp::spin_some(node); //allow data update from callback 
        naptime.sleep(); // wait for remainder of specified period; this loop rate is faster than 
        // the update rate of the 10Hz controller that specifies force_cmd 
        // however, simulator must advance each 10ms regardless 
    }
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0; // should never get here, unless ros2 core dies 
} 
