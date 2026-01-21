# Minimal Nodes ROS 2

This is a ROS 2 port of the minimal_nodes package from the learning_ros repository.

## Overview

This package contains five ROS 2 nodes demonstrating basic ROS 2 functionality:

1. **minimal_publisher** - Publishes Float64 messages to topic "topic1" at maximum rate (no sleep)
2. **sleepy_minimal_publisher** - Publishes Float64 messages to topic "topic1" at 1 Hz
3. **minimal_subscriber** - Subscribes to topic "topic1" and prints received messages
4. **minimal_simulator** - Simulates a simple physics system (F=ma), subscribes to "force_cmd" and publishes "velocity"
5. **minimal_controller** - A simple controller that subscribes to "velocity" and "vel_cmd", and publishes "force_cmd"

## ROS 1 to ROS 2 Conversion Notes

Key changes made during conversion:

- **Headers**: Changed from `ros/ros.h` to `rclcpp/rclcpp.hpp`
- **Message types**: Changed from `std_msgs::Float64` to `std_msgs::msg::Float64`
- **Node initialization**: Changed from `ros::init()` and `ros::NodeHandle` to `rclcpp::init()` and `rclcpp::Node::make_shared()`
- **Publishers**: Changed from `n.advertise<Type>("topic", buffer)` to `node->create_publisher<Type>("topic", qos)`
- **Subscribers**: Changed from `n.subscribe("topic", buffer, callback)` to `node->create_subscription<Type>("topic", qos, callback)`
- **Callback signatures**: Changed to use `SharedPtr` message types
- **Logging**: Changed from `ROS_INFO()` to `RCLCPP_INFO(logger, ...)`
- **Spin**: Changed from `ros::spin()`/`ros::spinOnce()` to `rclcpp::spin(node)`/`rclcpp::spin_some(node)`
- **Shutdown**: Added explicit `rclcpp::shutdown()` calls
- **Build system**: Changed from catkin to ament_cmake

## Building

1. Make sure you have ROS 2 installed and your workspace sourced:
   ```bash
   source /opt/ros/<your_ros2_distro>/setup.bash
   ```

2. If this is part of a workspace, ensure the workspace is built:
   ```bash
   cd <your_workspace>
   colcon build --packages-select minimal_nodes
   source install/setup.bash
   ```

## Running

### Terminal 1: Run a publisher
```bash
ros2 run minimal_nodes minimal_publisher
# or
ros2 run minimal_nodes sleepy_minimal_publisher
```

### Terminal 2: Run a subscriber
```bash
ros2 run minimal_nodes minimal_subscriber
```

### Terminal 3: Run the simulator/controller loop
```bash
# Terminal 3a: Run simulator
ros2 run minimal_nodes minimal_simulator

# Terminal 3b: Run controller
ros2 run minimal_nodes minimal_controller
```

### Monitor topics
```bash
ros2 topic echo /topic1
ros2 topic echo /velocity
ros2 topic echo /force_cmd
ros2 topic echo /vel_cmd
```
