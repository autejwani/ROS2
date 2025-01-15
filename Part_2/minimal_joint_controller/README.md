# Minimal Joint Controller

This package demonstrates how to communicate with Gazebo using ROS 2.

## Features
- Publishes velocity commands to a model in Gazebo.
- Sets an initial linear velocity of 1 m/s in the x-direction.
- Sets an initial angular velocity of 1 rad/s about the z-axis.

## Prerequisites
- **ROS 2** installed on your system.
- **Gazebo** (Ignition Gazebo) installed and configured.
- A workspace properly set up for building ROS 2 packages.

## Setup Instructions
1. Build the workspace using `colcon build`:

## Running the Example

### Step 1: Launch Gazebo with the Model
In a terminal, be in the models directory and launch Gazebo with the `ign gazebo -v 4 rect_prism.sdf` command:

### Step 2: Run the Minimal Joint Controller
In another terminal, run the ROS 2 node:
`ros2 run minimal_joint_controller minimal_joint_controller`

### Step 3: Observe Output
- The node will publish velocity commands to the `cmd_vel` topic of the model.
- Logs will print out the initial velocity values being published:
  - **Linear Velocity:** 1 m/s in the x-direction.
  - **Angular Velocity:** 1 rad/s about the z-axis.

### Gazebo Simulation and Terminal Output
![](images/Screenshot from 2025-01-15 15-23-07.png)
