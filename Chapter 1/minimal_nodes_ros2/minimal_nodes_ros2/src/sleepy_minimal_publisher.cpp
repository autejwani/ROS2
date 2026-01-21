#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS 2
    auto node = rclcpp::Node::make_shared("minimal_publisher2"); // name of this node will be "minimal_publisher2"
    
    // Create a publisher object that can talk to ROS 2
    auto my_publisher_object = node->create_publisher<std_msgs::msg::Float64>("topic1", 10);
    //"topic1" is the name of the topic to which we will publish
    // the "10" argument says to use a queue size of 10; could make larger, if expect network backups
    
    std_msgs::msg::Float64 input_float; //create a variable of type "Float64", 
    // as defined in: std_msgs/msg/Float64.hpp
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission
   
   rclcpp::Rate naptime(1.0); //create a ros object from the ros "Rate" class; 
   //set the sleep timer for 1Hz repetition rate (arg is in units of Hz)

    input_float.data = 0.0;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (rclcpp::ok()) 
    {
        // this loop has a sleep timer, so it will not consume excessive CPU time
        input_float.data = input_float.data + 0.001; //increment by 0.001 each iteration
        my_publisher_object->publish(input_float); // publish the value--of type Float64-- 
        //to the topic "topic1"
	//the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
	naptime.sleep(); 
    }
    
    rclcpp::shutdown(); // Shutdown ROS 2
    return 0;
}
