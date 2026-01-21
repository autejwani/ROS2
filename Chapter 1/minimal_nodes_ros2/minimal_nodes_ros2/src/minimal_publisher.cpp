#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS 2
    auto node = rclcpp::Node::make_shared("minimal_publisher"); // name of this node will be "minimal_publisher"
    
    // Create a publisher object that can talk to ROS 2
    auto my_publisher_object = node->create_publisher<std_msgs::msg::Float64>("topic1", 10);
    //"topic1" is the name of the topic to which we will publish
    // the "10" argument says to use a queue size of 10; could make larger, if expect network backups
    
    std_msgs::msg::Float64 input_float; //create a variable of type "Float64", 
    // as defined in: std_msgs/msg/Float64.hpp
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission

    input_float.data = 0.0;
    
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (rclcpp::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        input_float.data = input_float.data + 0.001; //increment by 0.001 each iteration
        my_publisher_object->publish(input_float); // publish the value--of type Float64-- 
        //to the topic "topic1" 
        rclcpp::spin_some(node); // Allow callbacks to be processed
    }
    
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
}
