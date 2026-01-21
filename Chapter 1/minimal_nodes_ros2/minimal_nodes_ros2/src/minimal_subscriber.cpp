#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

void myCallback(const std_msgs::msg::Float64::SharedPtr message_holder) 
{ 
  // the real work is done in this callback function 
  // it wakes up every time a new message is published on "topic1" 
  // Since this function is prompted by a message event, 
  //it does not consume CPU time polling for new data 
  // the RCLCPP_INFO() function is like a printf() function, except 
  // it publishes its output to the default rosout topic, which prevents 
  // slowing down this function for display calls, and it makes the 
  // data available for viewing and logging purposes 
  RCLCPP_INFO(rclcpp::get_logger("minimal_subscriber"), "received value is: %f", message_holder->data); 
  //really could do something interesting here with the received data...but all we do is print it 
} 

int main(int argc, char **argv) 
{ 
  rclcpp::init(argc, argv); // Initialize ROS 2
  auto node = rclcpp::Node::make_shared("minimal_subscriber"); // name this node 
  // when this compiled code is run, ROS will recognize it as a node called "minimal_subscriber" 
  
  //create a Subscriber object and have it subscribe to the topic "topic1" 
  // the function "myCallback" will wake up whenever a new message is published to topic1 
  // the real work is done inside the callback function 
  
  auto my_subscriber_object = node->create_subscription<std_msgs::msg::Float64>(
    "topic1", 10, myCallback); 

  rclcpp::spin(node); //this is essentially a "while(1)" statement, except it 
  // forces refreshing wakeups upon new data arrival 
  // main program essentially hangs here, but it must stay alive to keep the callback function alive 
  
  rclcpp::shutdown(); // Shutdown ROS 2
  return 0; // should never get here, unless ros2 core dies 
} 
